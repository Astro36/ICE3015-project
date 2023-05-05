#define F_CPU 16000000

#include <math.h>

#define _PORTA (*(volatile unsigned short*) 0x0400)
#define _PORTA_DIR (*(volatile unsigned short*) (0x0400 + 0x00))
#define _PORTA_OUT (*(volatile unsigned short*) (0x0400 + 0x04))
#define _PORTC (*(volatile unsigned short*) 0x0440)
#define _PORTC_DIR (*(volatile unsigned short*) (0x0440 + 0x00))
#define _PORTC_OUT (*(volatile unsigned short*) (0x0440 + 0x04))
#define _PORTF (*(volatile unsigned short*) 0x04A0)
#define _PORTF_DIR (*(volatile unsigned short*) (0x04A0 + 0x00))
#define _PORTF_OUT (*(volatile unsigned short*) (0x04A0 + 0x04))
#define _PORTMUX (*(volatile unsigned short*) 0x05E0)
#define _PORTMUX_TWISPIROUTEA (*(volatile unsigned short*) (0x05E0 + 0x03))
#define _PORTMUX_TWI0_ALT2_bm (unsigned char) 0b00100000

#define _TWI0 (*(volatile unsigned short*) 0x08A0)
#define _TWI0_MCTRLA (*(volatile unsigned short*) (0x08A0 + 0x03))
#define _TWI_ENABLE_bm (unsigned char) 0b00000001
#define _TWI0_MCTRLB (*(volatile unsigned short*) (0x08A0 + 0x04))
#define _TWI_ACKACT_bm (unsigned char) 0b00000100
#define _TWI_MCMD_RECVTRANS_gc (unsigned char) 0b00000010
#define _TWI_MCMD_STOP_gc (unsigned char) 0b00000011
#define _TWI0_MSTATUS (*(volatile unsigned short*) (0x08A0 + 0x05))
#define _TWI_RIF_bm (unsigned char) 0b10000000
#define _TWI_WIF_bm (unsigned char) 0b01000000
#define _TWI_RXACK_bm (unsigned char) 0b00010000
#define _TWI_ARBLOST_bm (unsigned char) 0b00001000
#define _TWI_BUSERR_bm (unsigned char) 0b00000100
#define _TWI_BUSSTATE_IDLE_bm (unsigned char) 0b00000001
#define _TWI0_MBAUD (*(volatile unsigned short*) (0x08A0 + 0x06))
#define _TWI0_MADDR (*(volatile unsigned short*) (0x08A0 + 0x07))
#define _TWI0_MDATA (*(volatile unsigned short*) (0x08A0 + 0x08))

#define _TWI_READ true
#define _TWI_WRITE false

#define MPU6050 0x68
#define MPU6050_ACCEL_XOUT_H 0x3b
#define MPU6050_PWR_MGMT_1 0x6b
#define MPU6050_CLOCK_PLL_XGYRO_bm 0b00000001

#define PI 3.141592
#define ALPHA 0.96
#define KP 5.0
#define KI 3.0
#define KD 3.0

#define MPU6050_ACC_RANGE 16384
#define MPU6050_GYRO_CONFIG 0x1b
#define MPU6050_GYRO_FS_250_bm 0b00000000
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_ACCEL_FS_2 0b00000000

void _twi_init();
bool _twi_start(unsigned char device, bool read);
void _twi_stop();
bool _twi_read(unsigned char* data, bool last);
bool _twi_write(unsigned char data);

void tcb0_init();
void tcb0_swap_pin();
void tcb0_set_duty();

void tcb1_init();
void tcb1_swap_pin();
void tcb1_set_duty();

void mpu6050_init();
void mpu6050_fetch(short* raw_ax, short* raw_ay, short* raw_az, short* raw_gx, short* raw_gy, short* raw_gz); // 자이로 가속도 정보 요청

void mx1508_init();
void mx1508_run(); // 모터 드라이버 속도 제어

void hcsr04_init();
void hcsr04_fetch(); // 거리 정보 요청

float mpu6050_offsets[6];
unsigned long previous_millis;
float angle_ax, angle_ay, angle_gx, angle_gy, angle_x, angle_y;
float pid_prev_err, pid_i_err;

void setup() {
    Serial.begin(9600); // debug
    mpu6050_init();
    hcsr04_init();
}

void loop() {
    // calculate `dt`
    unsigned long now = millis();
    float dt = (now - previous_millis) / 1000.0; // [sec]
    previous_millis = now;

    // fetch sensor data
    short raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
    mpu6050_fetch(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);

    // calculate angle by accel data
    float ax = (float) raw_ax - mpu6050_offsets[0];
    float ay = (float) raw_ay - mpu6050_offsets[1] + 16384;
    float az = (float) raw_az - mpu6050_offsets[2];
    angle_ax = atan(ay / sqrt(ax * ax + az * az)) * (180 / PI);
    angle_ay = atan(sqrt(ay * ay + az * az) / ax) * (180 / PI);
    Serial.println("loop accel:");
    Serial.println(angle_ax);
    Serial.println(angle_ay);

    // calculate angle by gyro data
    angle_gx += (float) (raw_gx - mpu6050_offsets[3]) / 131 * dt;
    angle_gy += (float) (raw_gy - mpu6050_offsets[4]) / 131 * dt;
    Serial.println("loop gyro:");
    Serial.println(angle_gx);
    Serial.println(angle_gy);

    // complementary filter
    angle_x = ALPHA * (angle_x + angle_gx * dt) + (1 - ALPHA) * angle_ax; // use only `angle_x`
    angle_y = ALPHA * (angle_y + angle_gy * dt) + (1 - ALPHA) * angle_ay; // unused

    // pid controller
    float sp = 0; // setpoint; fetch from HC-SR04
    float pid_err = sp - angle_x;
    pid_i_err += pid_err * dt;
    float pid_d_err = (pid_err - pid_prev_err) / dt;
    pid_prev_err = pid_err;
    float pv = KP * pid_err + KI * pid_i_err + KD * pid_d_err;
}

void _twi_init() {
    _PORTMUX_TWISPIROUTEA |= _PORTMUX_TWI0_ALT2_bm;

    unsigned int frequency = 400000; // 400kHz
    unsigned short t_rise = 300; // 300ns
    unsigned int baud = (F_CPU / frequency - F_CPU / 1000 / 1000 * t_rise / 1000 - 10) / 2;
    _TWI0_MBAUD = (unsigned char) baud;

    _TWI0_MCTRLA |= _TWI_ENABLE_bm;
    _TWI0_MSTATUS |= _TWI_BUSSTATE_IDLE_bm;
}

bool _twi_start(unsigned char device, bool read) {
    _TWI0_MADDR = device << 1 | read;

    while (!(_TWI0_MSTATUS & (_TWI_RIF_bm | _TWI_WIF_bm)))
        ;
    if (_TWI0_MSTATUS & (_TWI_ARBLOST_bm | _TWI_BUSERR_bm)) {
        return false; // ERROR
    }
    return !(_TWI0_MSTATUS & _TWI_RXACK_bm); // check ACK
}

void _twi_stop() {
    _TWI0_MCTRLB = _TWI_MCMD_STOP_gc;
}

bool _twi_read(unsigned char* data, bool last) {
    while (!(_TWI0_MSTATUS & (_TWI_RIF_bm | _TWI_WIF_bm)))
        ;
    if (_TWI0_MSTATUS & (_TWI_ARBLOST_bm | _TWI_BUSERR_bm)) {
        return false; // ERROR
    }

    *data = _TWI0_MDATA;
    if (last) {
        _TWI0_MCTRLB = _TWI_ACKACT_bm | _TWI_MCMD_STOP_gc; // send NACK
    } else {
        _TWI0_MCTRLB = _TWI_MCMD_RECVTRANS_gc; // send ACK
    }
    return true;
}

bool _twi_write(unsigned char data) {
    _TWI0_MDATA = data;
    _TWI0_MCTRLB = _TWI_MCMD_RECVTRANS_gc;

    while (!(_TWI0_MSTATUS & (_TWI_RIF_bm | _TWI_WIF_bm)))
        ;
    if (_TWI0_MSTATUS & (_TWI_ARBLOST_bm | _TWI_BUSERR_bm)) {
        return false; // ERROR
    }
    return !(_TWI0_MSTATUS & _TWI_RXACK_bm); // check ACK
}

void tcb0_init();
void tcb0_swap_pin();
void tcb0_set_duty();

void tcb1_init();
void tcb1_swap_pin();
void tcb1_set_duty();

void mpu6050_init() {
    _twi_init();

    // set clock source
    _twi_start(MPU6050, _TWI_WRITE);
    _twi_write(MPU6050_PWR_MGMT_1); // reg address
    _twi_stop();
    _twi_start(MPU6050, _TWI_READ);
    unsigned char pwr_mgmt_1;
    _twi_read(&pwr_mgmt_1, true); // reg value
    _twi_stop();
    pwr_mgmt_1 |= MPU6050_CLOCK_PLL_XGYRO_bm;
    _twi_start(MPU6050, _TWI_WRITE);
    _twi_write(MPU6050_PWR_MGMT_1); // reg address
    _twi_write(pwr_mgmt_1); // reg value
    _twi_stop();

    // init sensor offset
    short sum[6];
    for (int i = 0; i < 10; i += 1) {
        short raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
        mpu6050_fetch(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);
        sum[0] += raw_ax;
        sum[1] += raw_ay;
        sum[2] += raw_az;
        sum[3] += raw_gx;
        sum[4] += raw_gy;
        sum[5] += raw_gz;
    }
    for (int i = 0; i < 6; i += 1) {
        mpu6050_offsets[i] = (float) sum[i] / 10;
    }
}

void mpu6050_fetch(short* raw_ax, short* raw_ay, short* raw_az, short* raw_gx, short* raw_gy, short* raw_gz) {
    unsigned char buf[14];
    for (int i = 0; i < 14; i += 1) {
        _twi_start(MPU6050, _TWI_WRITE);
        _twi_write(MPU6050_ACCEL_XOUT_H + i); // reg address
        _twi_stop();
        _twi_start(MPU6050, _TWI_READ);
        _twi_read(&buf[i], i == 13); // reg value
        _twi_stop();
    }
    *raw_ax = (((short) buf[0]) << 8) | buf[1];
    *raw_ay = (((short) buf[2]) << 8) | buf[3];
    *raw_az = (((short) buf[4]) << 8) | buf[5];
    *raw_gx = (((short) buf[8]) << 8) | buf[9];
    *raw_gy = (((short) buf[10]) << 8) | buf[11];
    *raw_gz = (((short) buf[12]) << 8) | buf[13];
}

void mx1508_init();
void mx1508_run();

void hcsr04_init() {
    _PORTA_DIR |= 0b00000010; // PA1: OUTPUT, PA0: INPUT
    _PORTF_DIR |= 0b00001000; // PF3: OUTPUT, PF2: INPUT
}

void hcsr04_fetch() {
    _PORTA_OUT |= 0b00000010; // PA1: HIGH
    delayMicroseconds(10);
    _PORTA_OUT &= ~0b00000010; // PA1: LOW

    unsigned long width_left = 0;
    float distance_left = width_left / 58;

    _PORTF_OUT |= 0b00001000; // PF3: HIGH
    delayMicroseconds(10);
    _PORTF_OUT &= ~0b00001000; // PF3: LOW

    unsigned long width_right = 0;
    float distance_right = width_right / 58;
}
