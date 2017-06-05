#include "MPU9250.h"

int readReg16BigEndian(int i2c_handler, int reg_addr){
    return (wiringPiI2CReadReg8(i2c_handler, reg_addr) << 8) + wiringPiI2CReadReg8(i2c_handler, reg_addr + 1);
}

MPU9250::MPU9250(){
    m_mpu6050 = wiringPiI2CSetup(MPU9250_I2C_ADDR);
    // Enable Bypass mode (for magnetometer)
    wiringPiI2CWriteReg8(m_mpu6050, INT_PIN_CFG, 0x02);

    m_ak8963 = wiringPiI2CSetup(AK8963_I2C_ADDR);
    // Set magnetometer to continuous mesurement mode 1
    wiringPiI2CWriteReg8(m_ak8963, MAGN_CNTL, 0x02);

    // gyroscope bias
    gyro_x_bias = -0.81;
    gyro_y_bias = 1.27;
    gyro_z_bias = -0.64;

    m_accelSensitivity = 16384; // default (when accelerometer full scale range is +-2g)
    m_gyroSensitivity = 131.072; // default (when gyroscope full scale range is +-250degree/s)
    m_magnSensitivity = 0.6; // default (when magnetometer full scale range is +-4800uT)

    magn_asax = (readMagnASAX() - 128) / 256 + 1;
    magn_asay = (readMagnASAY() - 128) / 256 + 1;
    magn_asaz = (readMagnASAZ() - 128) / 256 + 1;
}

/*+++++++++++++++++ Accelerometer Definition +++++++++++++++++*/
void MPU9250::setupAccel(int cfg){
    wiringPiI2CWriteReg8(m_mpu6050, ACCEL_CONFIG, cfg);
    switch(cfg){
        case 0x00:
            m_accel_fullscale = 2;
            m_accelSensitivity = 65536.0 / 4;
            break;
        case 0x08:
            m_accel_fullscale = 4;
            m_accelSensitivity = 65536.0 / 8;
            break;
        case 0x10:
            m_accel_fullscale = 16;
            m_accelSensitivity = 65536.0 / 16;
            break;
        case 0x18:
            m_accel_fullscale = 32;
            m_accelSensitivity = 65536.0 / 32;
            break;
    }
}
double MPU9250::readAccelX(){
    double accel_x = readReg16BigEndian(m_mpu6050, ACCEL_X_H) / m_accelSensitivity;

    if(accel_x > m_accel_fullscale){
        return accel_x - (2 * m_accel_fullscale);
    }
    else{
        return accel_x;
    }
}
double MPU9250::readAccelY(){
    double accel_y = readReg16BigEndian(m_mpu6050, ACCEL_Y_H) / m_accelSensitivity;

    if(accel_y > m_accel_fullscale){
        return accel_y - (2 * m_accel_fullscale);
    }
    else{
        return accel_y;
    }
}
double MPU9250::readAccelZ(){
    double accel_z = readReg16BigEndian(m_mpu6050, ACCEL_Z_H) / m_accelSensitivity;

    if(accel_z > m_accel_fullscale){
        return accel_z - (2 * m_accel_fullscale);
    }
    else{
        return accel_z;
    }
}

/*+++++++++++++++++ Gyroscope Definition +++++++++++++++++*/
void MPU9250::setupGyro(int cfg){
    wiringPiI2CWriteReg8(m_mpu6050,GYRO_CONFIG, cfg);

    switch(cfg){
        case 0x00:
            m_gyro_fullscale = 250;
            m_gyroSensitivity = 65536.0 / (2 * 250);
            break;
        case 0x08:
            m_gyro_fullscale = 500;
            m_gyroSensitivity = 65536.0 / (2 * 500);
            break;
        case 0x10:
            m_gyro_fullscale = 1000;
            m_gyroSensitivity = 65536.0 / (2 * 1000);
            break;
        case 0x18:
            m_gyro_fullscale = 2000;
            m_gyroSensitivity = 65536.0 / (2 * 2000);
            break;
    }
}
double MPU9250::readGyroX(){
    double gyro_x = readReg16BigEndian(m_mpu6050, GYRO_X_H) / m_gyroSensitivity;

    if(gyro_x > m_gyro_fullscale){
        gyro_x -= (2 * m_gyro_fullscale);
    }

    return gyro_x - gyro_x_bias;
}
double MPU9250::readGyroY(){
    double gyro_y = readReg16BigEndian(m_mpu6050, GYRO_Y_H) / m_gyroSensitivity;

    if(gyro_y > m_gyro_fullscale){
        gyro_y -= (2 * m_gyro_fullscale);
    }

    return gyro_y - gyro_y_bias;
}
double MPU9250::readGyroZ(){
    double gyro_z = readReg16BigEndian(m_mpu6050, GYRO_Z_H) / m_gyroSensitivity;

    if(gyro_z > m_gyro_fullscale){
        gyro_z -= (2 * m_gyro_fullscale);
    }

    return gyro_z - gyro_z_bias;
}

/*+++++++++++++++++ Magnetometer Definition +++++++++++++++++*/
double MPU9250::readMagnX(){
    return wiringPiI2CReadReg16(m_ak8963, MAGN_X_L) * m_magnSensitivity * magn_asax;
}
double MPU9250::readMagnY(){
    return wiringPiI2CReadReg16(m_ak8963, MAGN_Y_L) * m_magnSensitivity * magn_asay;
}
double MPU9250::readMagnZ(){
    return wiringPiI2CReadReg16(m_ak8963, MAGN_Z_L) * m_magnSensitivity * magn_asaz;
}

/** Get sensitivity adjustment from Fuse ROM **/
double MPU9250::readMagnASAX(){
    return wiringPiI2CReadReg8(m_ak8963, MAGN_ASAX);
}
double MPU9250::readMagnASAY(){
    return wiringPiI2CReadReg8(m_ak8963, MAGN_ASAY);
}
double MPU9250::readMagnASAZ(){
    return wiringPiI2CReadReg8(m_ak8963, MAGN_ASAZ);
}

/** Read ST2 **/
int MPU9250::readMagnST2(){
    return wiringPiI2CReadReg8(m_ak8963, MAGN_ST2);
}
