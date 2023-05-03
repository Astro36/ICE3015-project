#define F_CPU 160000000

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

#define MPU6050_ACC_RANGE 16384
#define MPU6050_AX_OFFSET (short) 0
#define MPU6050_AY_OFFSET (short) 0
#define MPU6050_AZ_OFFSET (short) 0
#define MPU6050_GX_OFFSET (short) 0
#define MPU6050_GY_OFFSET (short) 0
#define MPU6050_GZ_OFFSET (short) 0

#define MPU6050_GYRO_CONFIG 0x1b
#define MPU6050_GYRO_FS_250_bm 0b00000000
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_ACCEL_FS_2 0b00000000

#define MPU6050_ACCEL_XOUT_H 0x3b

#define MPU6050_PWR_MGMT_1 0x6b
#define MPU6050_CLOCK_PLL_XGYRO_bm 0b00000001

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
void mpu6050_fetch(float* ax, float* ay, float* az, float* gx, float* gy, float* gz); // 자이로 가속도 정보 요청

void mx1508_init();
void mx1508_run(); // 모터 드라이버 속도 제어

void hcsr04_init();
void hcsr04_fetch(); // 거리 정보 요청

void complementary_filter(float* yaw, float* pitch, float* roll, float ax, float ay, float az, float gx, float gy, float gz);
void pid_controller();

void setup() {
    _twi_init();
    mpu6050_init();
}

void loop() {
    float ax, ay, az, gx, gy, gz;
    mpu6050_fetch(&ax, &ay, &az, &gx, &gy, &gz);

    float yaw, pitch, roll;
    complementary_filter(&yaw, &pitch, &roll, ax, ay, az, gx, gy, gz);
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
    // clock source
    _twi_start(0x00, _TWI_WRITE);
    _twi_write(MPU6050_PWR_MGMT_1); // reg address
    _twi_stop();
    _twi_start(0x00, _TWI_READ);
    unsigned char pwr_mgmt_1;
    _twi_read(&pwr_mgmt_1, true);
    _twi_stop();
    pwr_mgmt_1 |= MPU6050_CLOCK_PLL_XGYRO_bm;
    _twi_start(0x00, _TWI_WRITE);
    _twi_write(MPU6050_PWR_MGMT_1); // reg address
    _twi_write(pwr_mgmt_1); // reg value
    _twi_stop();
}

void mpu6050_fetch(unsigned char dev, float* ax, float* ay, float* az, float* gx, float* gy, float* gz) {
    unsigned char buf[14];
    for (int i = 0; i < 14; i += 1) {
        _twi_start(0x00, _TWI_WRITE);
        _twi_write(MPU6050_ACCEL_XOUT_H + i); // reg address
        _twi_stop();
        _twi_start(0x00, _TWI_READ);
        _twi_read(&buf[i], i == 13);
        _twi_stop();
    }
    short raw_ax = (((short) buf[0]) << 8) | buf[1];
    short raw_ay = (((short) buf[2]) << 8) | buf[3];
    short raw_az = (((short) buf[4]) << 8) | buf[5];
    short raw_gx = (((short) buf[8]) << 8) | buf[9];
    short raw_gy = (((short) buf[10]) << 8) | buf[11];
    short raw_gz = (((short) buf[12]) << 8) | buf[13];

    *ax = (float) raw_ax / MPU6050_ACC_RANGE;
    *ay = (float) raw_ay / MPU6050_ACC_RANGE;
    *az = (float) raw_az / MPU6050_ACC_RANGE;
}

void mx1508_init();
void mx1508_run();

void hcsr04_init();
void hcsr04_fetch();

void complementary_filter(float* yaw, float* pitch, float* roll, float ax, float ay, float az, float gx, float gy, float gz) {
}
void pid_controller();
