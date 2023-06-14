#define F_CPU 16000000

#include "Wire.h"

#define _PORTMUX_TWISPIROUTEA (*(volatile unsigned char*) (0x05E0 + 0x03))
#define _PORTMUX_TWI0_ALT2_bm ((unsigned char) 0b00100000)

#define MPU6050 0x68
#define MPU6050_ACCEL_XOUT_H 0x3b
#define MPU6050_PWR_MGMT_1 0x6b
#define MPU6050_CLOCK_PLL_XGYRO_bm 0b00000001

void setup() {
    Serial1.begin(9600);

    Wire.begin();
    _PORTMUX_TWISPIROUTEA |= _PORTMUX_TWI0_ALT2_bm;

    Wire.beginTransmission(MPU6050);
    Wire.write(MPU6050_PWR_MGMT_1);
    Wire.write(0);
    Wire.endTransmission(true);
}

void loop() {
    Wire.beginTransmission(MPU6050);
    Wire.write(MPU6050_ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true);

    short raw_ax = Wire.read() << 8 | Wire.read();
    short raw_ay = Wire.read() << 8 | Wire.read();
    short raw_az = Wire.read() << 8 | Wire.read();
    short raw_tmp = Wire.read() << 8 | Wire.read();
    short raw_gx = Wire.read() << 8 | Wire.read();
    short raw_gy = Wire.read() << 8 | Wire.read();
    short raw_gz = Wire.read() << 8 | Wire.read();

    Serial1.println(raw_ax);
    Serial1.println(raw_ay);
    Serial1.println(raw_az);
    Serial1.println(raw_tmp);
    Serial1.println(raw_gx);
    Serial1.println(raw_gy);
    Serial1.println(raw_gz);

    delay(1000);
}
