#define F_CPU 16000000

#include <math.h>

#define _PORTA (*(volatile unsigned char*) 0x0400)
#define _PORTA_DIR (*(volatile unsigned char*) (0x0400 + 0x00))
#define _PORTA_OUT (*(volatile unsigned char*) (0x0400 + 0x04))
#define _PORTC (*(volatile unsigned char*) 0x0440)
#define _PORTC_DIR (*(volatile unsigned char*) (0x0440 + 0x00))
#define _PORTC_OUT (*(volatile unsigned char*) (0x0440 + 0x04))
#define _PORTC_PIN2CTRL (*(volatile unsigned char*) (0x0440 + 0x12))
#define _PORTC_PIN2CTRL_PULLUPEN_bm ((unsigned char) 0b00001000)
#define _PORTC_PIN3CTRL (*(volatile unsigned char*) (0x0440 + 0x13))
#define _PORTC_PIN3CTRL_PULLUPEN_bm ((unsigned char) 0b00001000)
#define _PORTF (*(volatile unsigned char*) 0x04A0)
#define _PORTF_DIR (*(volatile unsigned char*) (0x04A0 + 0x00))
#define _PORTF_OUT (*(volatile unsigned char*) (0x04A0 + 0x04))
#define _PORTMUX (*(volatile unsigned char*) 0x05E0)
#define _PORTMUX_TWISPIROUTEA (*(volatile unsigned char*) (0x05E0 + 0x03))
#define _PORTMUX_TWI0_ALT2_bm ((unsigned char) 0b00100000)
#define _PORTMUX_TCBROUTEA (*(volatile unsigned char*) (0x05E0 + 0x05))
#define _PORTMUX_TCB1_ALT1_gc ((unsigned char) 0b00000010)
#define _PORTMUX_TCB0_ALT1_gc ((unsigned char) 0b00000001)

#define _TWI0 (*(volatile unsigned char*) 0x08A0)
#define _TWI0_MCTRLA (*(volatile unsigned char*) (0x08A0 + 0x03))
#define _TWI_ENABLE_bm ((unsigned char) 0b00000001)
#define _TWI0_MCTRLB (*(volatile unsigned char*) (0x08A0 + 0x04))
#define _TWI_ACKACT_NACK_gc ((unsigned char) 0b00000100)
#define _TWI_MCMD_RECVTRANS_gc ((unsigned char) 0b00000010)
#define _TWI_MCMD_STOP_gc ((unsigned char) 0b00000011)
#define _TWI0_MSTATUS (*(volatile unsigned char*) (0x08A0 + 0x05))
#define _TWI_RIF_bm ((unsigned char) 0b10000000)
#define _TWI_WIF_bm ((unsigned char) 0b01000000)
#define _TWI_RXACK_bm ((unsigned char) 0b00010000)
#define _TWI_ARBLOST_bm ((unsigned char) 0b00001000)
#define _TWI_BUSERR_bm ((unsigned char) 0b00000100)
#define _TWI_BUSSTATE_IDLE_gc ((unsigned char) 0b00000001)
#define _TWI0_MBAUD (*(volatile unsigned char*) (0x08A0 + 0x06))
#define _TWI0_MADDR (*(volatile unsigned char*) (0x08A0 + 0x07))
#define _TWI0_MDATA (*(volatile unsigned char*) (0x08A0 + 0x08))

#define _TCB0 (*(volatile unsigned char*) 0x0A80)
#define _TCB0_CTRLA (*(volatile unsigned char*) (0x0A80 + 0x00))
#define _TCB_CLKSEL_CLKDIV2_gc ((unsigned char) 0b00000010)
#define _TCB_ENABLE_bm ((unsigned char) 0b00000001)
#define _TCB0_CTRLB (*(volatile unsigned char*) (0x0A80 + 0x01))
#define _TCB_CCMPEN_bm ((unsigned char) 0b00000111)
#define _TCB_CNTMODE_PWM8_gc (unsigned char) 0b00010000
#define _TCB0_CCMPL (*(volatile unsigned char*) (0x0A80 + 0x0C))
#define _TCB0_CCMPH (*(volatile unsigned char*) (0x0A80 + 0x0D))
#define _TCB1 (*(volatile unsigned char*) 0x0A90)
#define _TCB1_CTRLA (*(volatile unsigned char*) (0x0A90 + 0x00))
#define _TCB1_CTRLB (*(volatile unsigned char*) (0x0A90 + 0x01))
#define _TCB1_CCMPL (*(volatile unsigned char*) (0x0A90 + 0x0C))
#define _TCB1_CCMPH (*(volatile unsigned char*) (0x0A90 + 0x0D))

#define _TWI_READ true
#define _TWI_WRITE false

#define MPU6050 0x68
#define MPU6050_ACCEL_XOUT_H 0x3b
#define MPU6050_PWR_MGMT_1 0x6b
#define MPU6050_CLOCK_PLL_XGYRO_bm 0b00000001

#define PI 3.141592
#define KP 5.0
#define KD 1.5

// #define MPU6050_ACC_RANGE 16384
// #define MPU6050_GYRO_CONFIG 0x1b
// #define MPU6050_GYRO_FS_250_bm 0b00000000
// #define MPU6050_ACCEL_CONFIG 0x1c
// #define MPU6050_ACCEL_FS_2 0b00000000

void _twi_init();
bool _twi_start(unsigned char device, bool read);
void _twi_stop();
bool _twi_read(unsigned char* data, bool last);
bool _twi_write(unsigned char data);

void _tcb0_init();
void _tcb0_default_pin();
void _tcb0_alt_pin();
void _tcb0_set_duty(unsigned char duty);
void _tcb1_init();
void _tcb1_default_pin();
void _tcb1_alt_pin();
void _tcb1_set_duty(unsigned char duty);

void mpu6050_init();
void mpu6050_fetch(short* raw_ax, short* raw_ay, short* raw_az, short* raw_gx, short* raw_gy, short* raw_gz); // 자이로 가속도 정보 요청

void mx1508_init();
void mx1508_left_run(); // 모터 드라이버 속도 제어
void mx1508_right_run(); // 모터 드라이버 속도 제어

// void hcsr04_init();
// void hcsr04_fetch(); // 거리 정보 요청

short mpu6050_offsets[6];
unsigned long previous_millis;
float angle_ax = 0, angle_gx = 0, angle_x = 0;
float pid_prev_err, pid_i_err;

char debug[7];

char* float2str(float i) {
    sprintf(debug, "%6d/1000", (int) i * 1000);
    return debug;
}

char* short2str(short i) {
    sprintf(debug, "%6d", i);
    return debug;
}

int main() {
    Serial1.begin(9600); // debug
    mpu6050_init();
    mx1508_init();
    // hcsr04_init();

    while (true) {
        // calculate `dt`
        unsigned long now = millis();
        float dt = (now - previous_millis) / 1000.0; // [sec]
        previous_millis = now;

        // fetch sensor data
        short raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
        mpu6050_fetch(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);

        // calculate angle by accel data
        float ax = raw_ax - mpu6050_offsets[0];
        float ay = raw_ay - mpu6050_offsets[1];
        float az = raw_az - mpu6050_offsets[2];

        angle_x = atan(ay / sqrt(ax * ax + az * az)) * (180 / PI);

        // pid controller
        float sp = 0; // setpoint; fetch from HC-SR04
        float pid_err = sp - angle_x;
        float pid_d_err = (pid_err - pid_prev_err) / dt;
        pid_prev_err = pid_err;
        float pv = KP * pid_err + KD * pid_d_err;

        // debug
        Serial1.print("angle_x=");
        Serial1.print(float2str(angle_x));
        Serial1.print(", P=");
        Serial1.print(float2str(KP * pid_err));
        Serial1.print(", D=");
        Serial1.print(float2str(KD * pid_d_err));
        Serial1.print(", pv=");
        Serial1.print(float2str(pv));
        Serial1.println();

        float max_pv = 64;
        if (pv > max_pv) {
            pv = max_pv;
        } else if (pv < -max_pv) {
            pv = -max_pv;
        }

        // run pwm motor
        mx1508_left_run(pv);
        mx1508_right_run(pv);
    }
    return 0;
}

void _twi_init() {
    _PORTMUX_TWISPIROUTEA |= _PORTMUX_TWI0_ALT2_bm;
    _PORTC_PIN2CTRL |= _PORTC_PIN2CTRL_PULLUPEN_bm; // PC2: PULLUP
    _PORTC_PIN3CTRL |= _PORTC_PIN3CTRL_PULLUPEN_bm; // PC3: PULLUP

    unsigned int frequency = 400000; // 400kHz
    unsigned short t_rise = 300; // 300ns
    unsigned int baud = (F_CPU / frequency - F_CPU / 1000 / 1000 * t_rise / 1000 - 10) / 2;
    _TWI0_MBAUD = (unsigned char) baud;

    _TWI0_MCTRLA = _TWI_ENABLE_bm;
    _TWI0_MSTATUS = _TWI_BUSSTATE_IDLE_gc;
}

bool _twi_start(unsigned char device, bool read) {
    _TWI0_MADDR = device << 1 | (read ? 1 : 0);

    while (!(_TWI0_MSTATUS & (_TWI_WIF_bm | _TWI_RIF_bm)))
        ;

    if (_TWI0_MSTATUS & _TWI_ARBLOST_bm) {
        while (!(_TWI0_MSTATUS & _TWI_BUSSTATE_IDLE_gc))
            ;
        return false;
    }
    if (_TWI0_MSTATUS & _TWI_RXACK_bm) {
        _TWI0_MCTRLB |= _TWI_MCMD_STOP_gc;
        while (!(_TWI0_MSTATUS & _TWI_BUSSTATE_IDLE_gc))
            ;
        return false;
    }
    return true;
}

void _twi_stop() {
    _TWI0_MCTRLB |= _TWI_MCMD_STOP_gc;
    while (!(_TWI0_MSTATUS & _TWI_BUSSTATE_IDLE_gc))
        ;
}

bool _twi_read(unsigned char* data, bool last) {
    while (!(_TWI0_MSTATUS & _TWI_RIF_bm))
        ;

    *data = _TWI0_MDATA;
    if (last) {
        _TWI0_MCTRLB = _TWI_ACKACT_NACK_gc; // send NACK
    } else {
        _TWI0_MCTRLB = _TWI_MCMD_RECVTRANS_gc; // send ACK
    }
    return true;
}

bool _twi_write(unsigned char data) {
    _TWI0_MCTRLB = _TWI_MCMD_RECVTRANS_gc;
    _TWI0_MDATA = data;

    while (!(_TWI0_MSTATUS & _TWI_WIF_bm))
        ;
    if (_TWI0_MSTATUS & (_TWI_ARBLOST_bm | _TWI_BUSERR_bm)) {
        return false; // ERROR
    }
    return !(_TWI0_MSTATUS & _TWI_RXACK_bm); // check ACK
}

void _tcb0_init() {
    _PORTA_DIR |= 0b00000100; // PA2: OUTPUT
    _PORTF_DIR |= 0b00010000; // PF4: OUTPUT
    _TCB0_CCMPH = 0x00; // Duty
    _TCB0_CCMPL = 0xff; // TOP
    _TCB0_CTRLA |= _TCB_CLKSEL_CLKDIV2_gc | _TCB_ENABLE_bm;
    _TCB0_CTRLB |= _TCB_CCMPEN_bm | _TCB_CNTMODE_PWM8_gc;
}

void _tcb0_default_pin() {
    _PORTMUX_TCBROUTEA &= ~_PORTMUX_TCB0_ALT1_gc;
    _PORTF_OUT &= ~0b00010000; // PF4: LOW
}

void _tcb0_alt_pin() {
    _PORTMUX_TCBROUTEA |= _PORTMUX_TCB0_ALT1_gc;
    _PORTA_OUT &= ~0b00000100; // PA2: LOW
}

void _tcb0_set_duty(unsigned char duty) {
    _TCB0_CCMPH = duty;
}

void _tcb1_init() {
    _PORTA_DIR |= 0b00001000; // PA3: OUTPUT
    _PORTF_DIR |= 0b00100000; // PF5: OUTPUT
    _TCB1_CCMPH = 0x00; // Duty
    _TCB1_CCMPL = 0xff; // TOP
    _TCB1_CTRLA |= _TCB_CLKSEL_CLKDIV2_gc | _TCB_ENABLE_bm;
    _TCB1_CTRLB |= _TCB_CCMPEN_bm | _TCB_CNTMODE_PWM8_gc;
}

void _tcb1_default_pin() {
    _PORTMUX_TCBROUTEA &= ~_PORTMUX_TCB1_ALT1_gc;
    _PORTF_OUT &= ~0b00100000; // PF5: LOW
}

void _tcb1_alt_pin() {
    _PORTMUX_TCBROUTEA |= _PORTMUX_TCB1_ALT1_gc;
    _PORTA_OUT &= ~0b00001000; // PA3: LOW
}

void _tcb1_set_duty(unsigned char duty) {
    _TCB1_CCMPH = duty;
}

void mpu6050_init() {
    _twi_init();

    _twi_start(MPU6050, _TWI_WRITE);
    _twi_write(MPU6050_PWR_MGMT_1); // reg address
    _twi_write(0); // reg value
    _twi_stop();

    // init sensor offset
    short sum[6] = { 0 };
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
        mpu6050_offsets[i] = sum[i] / 10;
    }
}

void mpu6050_fetch(short* raw_ax, short* raw_ay, short* raw_az, short* raw_gx, short* raw_gy, short* raw_gz) {
    unsigned char buf[14];
    for (int i = 0; i < 14; i += 1) {
        _twi_start(MPU6050, _TWI_WRITE);
        _twi_write(MPU6050_ACCEL_XOUT_H + i); // reg address
        _twi_stop();
        _twi_start(MPU6050, _TWI_READ);
        _twi_read(&buf[i], true); // reg value
        _twi_stop();
    }
    *raw_ax = (buf[0] << 8) | buf[1];
    *raw_ay = (buf[2] << 8) | buf[3];
    *raw_az = (buf[4] << 8) | buf[5];
    *raw_gx = (buf[8] << 8) | buf[9];
    *raw_gy = (buf[10] << 8) | buf[11];
    *raw_gz = (buf[12] << 8) | buf[13];
}

void mx1508_init() {
    _tcb0_init();
    _tcb1_init();
}

void mx1508_left_run(float power) {
    if (power >= 0) {
        _tcb0_default_pin();
        _tcb0_set_duty((unsigned char) power);
    } else {
        _tcb0_alt_pin();
        _tcb0_set_duty((unsigned char) -power);
    }
}

void mx1508_right_run(float power) {
    if (power >= 0) {
        _tcb1_default_pin();
        _tcb1_set_duty((unsigned char) power);
    } else {
        _tcb1_alt_pin();
        _tcb1_set_duty((unsigned char) -power);
    }
}

// void hcsr04_init() {
//     _PORTA_DIR |= 0b00000010; // PA1: OUTPUT, PA0: INPUT
//     _PORTF_DIR |= 0b00001000; // PF3: OUTPUT, PF2: INPUT
// }

// void hcsr04_fetch() {
//     _PORTA_OUT |= 0b00000010; // PA1: HIGH
//     delayMicroseconds(10);
//     _PORTA_OUT &= ~0b00000010; // PA1: LOW

//     unsigned long width_left = 0;
//     float distance_left = width_left / 58;

//     _PORTF_OUT |= 0b00001000; // PF3: HIGH
//     delayMicroseconds(10);
//     _PORTF_OUT &= ~0b00001000; // PF3: LOW

//     unsigned long width_right = 0;
//     float distance_right = width_right / 58;
// }
