#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include "MPU9250.h"
#include "KalmanFilter.h"
#include "PID.h"
#include "CascadePID.h"
#include "tools.h"

int kalmantest()
{
    MPU9250 mpu9250;
    KalmanFilter kalman_pitch;
    KalmanFilter kalman_roll;
    PID pid_pitch;
    PID pid_roll;
    CascadePID cpid_pitch;
    CascadePID cpid_roll;
    mpu9250.setupGyro(mpu9250.GYRO_FS_500);

    int fd = open("kalman.txt", O_CREAT | O_RDWR | O_TRUNC);
    int count = 0;
    char buff[1024];

    double lasttime, nowtime;
    double pitch = 0.0, roll = 0.0, yaw = 0.0;
    double pitch_gyro = 0.0, roll_gyro = 0.0;
    double pitch_kalman = 0.0, roll_kalman = 0.0;

    double accel_x = 0;
    double accel_y = 0;
    double accel_z = 0;

    double gyro_x = 0;
    double gyro_y = 0;
    double gyro_z = 0;
    double gyro_x_bias = -0.81;
    double gyro_y_bias = 1.27;
    double gyro_z_bias = -0.64;

    double magn_x = 0;
    double magn_y = 0;
    double magn_z = 0;

    double hx = 0,hy = 0;

    double output_pitch;
    double output_roll;

    delay(1000);

    lasttime = now();

    accel_x = mpu9250.readAccelX();
    accel_x = accel_x > 2 ? accel_x - 4 : accel_x;
    accel_y = mpu9250.readAccelY();
    accel_y = accel_y > 2 ? accel_y - 4 : accel_y;
    accel_z = mpu9250.readAccelZ();
    accel_z = accel_z > 2 ? accel_z - 4 : accel_z;

    pitch_gyro = atan(accel_x / sqrt(pow(accel_y, 2) + pow(accel_z, 2))) * (180/PI);
    roll_gyro = atan(accel_y / sqrt(pow(accel_x, 2) + pow(accel_z, 2))) * (180/PI);

    kalman_pitch.setAngle(pitch_gyro);
    kalman_roll.setAngle(roll_gyro);

    // while(count!=500){
    while(TRUE){
        accel_x = mpu9250.readAccelX();
        accel_x = accel_x > 2 ? accel_x - 4 : accel_x;
        accel_y = mpu9250.readAccelY();
        accel_y = accel_y > 2 ? accel_y - 4 : accel_y;
        accel_z = mpu9250.readAccelZ();
        accel_z = accel_z > 2 ? accel_z - 4 : accel_z;

        nowtime = now();

        gyro_x = mpu9250.readGyroX();
        gyro_x = gyro_x > 500 ? gyro_x - 1000 : gyro_x;
        gyro_y = mpu9250.readGyroY();
        gyro_y = gyro_y > 500 ? gyro_y - 1000 : gyro_y;
        gyro_z = mpu9250.readGyroZ();
        gyro_z = gyro_z > 500 ? gyro_z - 1000 : gyro_z;

        magn_x = mpu9250.readMagnX();
        magn_y = mpu9250.readMagnY();
        magn_z = mpu9250.readMagnZ();
        mpu9250.readMagnST2();

        pitch_gyro += (gyro_y) * ((nowtime - lasttime));
        roll_gyro += (gyro_x) * ((nowtime - lasttime));

        pitch = atan(accel_x / sqrt(pow(accel_y, 2) + pow(accel_z, 2)));
        roll = atan(accel_y / sqrt(pow(accel_x, 2) + pow(accel_z, 2)));

        hy = magn_y * cos(pitch) + magn_x * sin(pitch) * sin(roll) - magn_z * sin(pitch) * cos(roll);
        hx = magn_x * cos(roll) + magn_z * sin(roll);

        pitch *= (180/PI);
        roll *= (180/PI);
        yaw = atan(magn_y/magn_x) * (180/PI);

        pitch_kalman = kalman_pitch.updateAngle(pitch, gyro_y, nowtime - lasttime);
        roll_kalman = kalman_roll.updateAngle(roll, gyro_x, nowtime - lasttime);

        lasttime = nowtime;
/*
        output_pitch = pid_pitch.calculate(pitch_kalman);
        output_roll = pid_roll.calculate(roll_kalman);
*/

        output_pitch = cpid_pitch.calculate(pitch_kalman, abs(gyro_y));
        output_roll = cpid_roll.calculate(roll_kalman, abs(gyro_x));
/*
        printf("magn_x: %lf\tmagn_y: %lf\tmagn_z: %lf\n", magn_x, magn_y, magn_z);
        printf("pitch: %lf\troll: %lf\tyaw: %lf\n", pitch, roll, yaw);
*/

        printf("pitch: %lf\toutput_pitch: %lf\n", pitch_kalman, output_pitch);
        printf("roll: %lf\toutput_roll: %lf\n", roll_kalman, output_roll);

        sprintf(buff, "%d,%lf,%lf\n", count, pitch_kalman, roll_kalman);
        printf("%d#%s\n",strlen(buff), buff);
        write(fd, buff, strlen(buff));
        fflush(NULL);
        count++;
/*
        printf("pitch:\t\t %lf\n", pitch);
        printf("pitch_gyro:\t %lf\n", pitch_gyro);
        printf("pitch_kalman: %lf\n", pitch_kalman);

        printf("roll:\t\t %lf\n", roll);
        printf("roll_gyro:\t %lf\n", roll_gyro);
        printf("roll_kalman: %lf\n", roll_kalman);

        printf("---------------------------------------\n");
*/
        delay(10);
    }

//    close(fd);

    return 0;
}


int PIDtest(){
    //PID pid;
    CascadePID cpid;
    double error = 0.0;
    double output = 0.0;

    delay(3000);

    printf("set error to 30\n");
    error = 30;
    while(error >= 0){
        // output = pid.calculate(error);
        output = cpid.calculate(error, 0);
        printf("Error: %lf\tOutput: %lf\n", error, output);
        error -= 2;
        delay(200);
    }
/*
    while(1){
        output = pid.calculate(error);
        printf("Error: %lf\tIntegral: %lf\tOutput: %lf\n", error, pid.getIntegral(), output);
        delay(20);
    }
*/
    return 0;
}

int main0(){
    int type = 0;
    switch(type){
        case 0:
            kalmantest();
            break;
        case 1:
            PIDtest();
            break;
    }

    return 0;
}
