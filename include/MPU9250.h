#include <wiringPi.h>
#include <wiringPiI2C.h>

int readReg16BigEndian(int i2c_handler, int reg_addr);

class MPU9250
{
    private:
        static const int MPU9250_I2C_ADDR = 0x68;
        static const int AK8963_I2C_ADDR = 0x0C;    // Magnetometer i2c address

        static const int INT_PIN_CFG = 0x37; // INT pin/bypass enable configuration Register

        /*+++++++++++++++++ Accelerometer Definition +++++++++++++++++*/
        static const int ACCEL_CONFIG = 0x1C;   // accelerometer config register
        static const int ACCEL_CONFIG2 = 0x1D;

        static const int ACCEL_X_H = 0x3B;
        static const int ACCEL_X_L = 0x3C;
        static const int ACCEL_Y_H = 0x3D;
        static const int ACCEL_Y_L = 0x3E;
        static const int ACCEL_Z_H = 0x3F;
        static const int ACCEL_Z_L = 0x40;

        /*+++++++++++++++++ Gyroscope Definition +++++++++++++++++*/
        static const int GYRO_CONFIG = 0x1B;    // gyroscope config register

        static const int GYRO_X_H = 0x43;
        static const int GYRO_X_L = 0x44;
        static const int GYRO_Y_H = 0x45;
        static const int GYRO_Y_L = 0x46;
        static const int GYRO_Z_H = 0x47;
        static const int GYRO_Z_L = 0x48;

        /*+++++++++++++++++ Magnetometer Definition +++++++++++++++++*/
        static const int MAGN_X_L = 0x03;
        static const int MAGN_X_H = 0x04;
        static const int MAGN_Y_L = 0x05;
        static const int MAGN_Y_H = 0x06;
        static const int MAGN_Z_L = 0x07;
        static const int MAGN_Z_H = 0x08;

        static const int MAGN_ST2 = 0x09; // Status 2 Register
        static const int MAGN_CNTL = 0x0A;  // Control Register

        static const int MAGN_ASAX = 0x10;
        static const int MAGN_ASAY = 0x11;
        static const int MAGN_ASAZ = 0x12;

        int m_mpu6050;
        int m_ak8963;

        int m_accel_fullscale;
        int m_gyro_fullscale;

        double m_accelSensitivity; // units(LSB/g)
        double m_gyroSensitivity; // units(LSB/degree/s)
        double m_magnSensitivity; // units(uT/LSB)

        double magn_asax;
        double magn_asay;
        double magn_asaz;

        double gyro_x_bias;
        double gyro_y_bias;
        double gyro_z_bias;

    public:
        MPU9250();

        static const int ACCEL_FS_2 = 0x00; // full scale +-2g
        static const int ACCEL_FS_4 = 0X08; // full scale +-4g
        static const int ACCEL_FS_8 = 0X10; // full scale +-8g
        static const int ACCEL_FS_16 = 0x18; //full scale +-16g

        static const int GYRO_FS_250 = 0X00;   // full scale 250dps
        static const int GYRO_FS_500 = 0x08;   // full scale 500dps
        static const int GYRO_FS_1000 = 0x10;  // full scale 1000dps
        static const int GYRO_FS_2000 = 0x18;  // full scale 2000dps

        /*+++++++++++++++++ Gyroscope +++++++++++++++++*/

        void setupGyro(int cfg);
        double readGyroX();
        double readGyroY();
        double readGyroZ();

        /*+++++++++++++++++ Accelerometer +++++++++++++++++*/

        void setupAccel(int cfg);
        double readAccelX();
        double readAccelY();
        double readAccelZ();

        /*+++++++++++++++++ Magnetometer +++++++++++++++++*/

        double readMagnX();
        double readMagnY();
        double readMagnZ();

        double readMagnASAX();
        double readMagnASAY();
        double readMagnASAZ();
        // Read magnetometer status2 register.
        // Attention! In continuous measurement mode,
        // you need to call this function every time
        // after finish read measurement data
        int readMagnST2();
};
