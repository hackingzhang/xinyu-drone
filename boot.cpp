#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>

#include "Echo.h"
#include "CtrlRecv.h"
#include "MPU9250.h"
#include "ESCDriver.h"
#include "KalmanFilter.h"
#include "PID.h"
#include "CascadePID.h"
#include "tools.h"

static const char* LEFT = "left";
static const char* RIGHT = "right";
static const char* ACTION = "action";
static const char* DELIM = ",";

static const int THR_MIN = ESCDriver::DC_IDLE;
static const int THR_BASE = 0.6 * (ESCDriver::DC_MAX - ESCDriver::DC_IDLE);
static const int THR_MAX = ESCDriver::DC_IDLE + THR_BASE;

const static int MOTOR_MAX = 23;
const static int MOTOR_MIN = -23;

const static double PITCH_BIAS = 0.62;
const static double ROLL_BIAS = 0.35;

// max angle (degree)
static const double ANGLE_MAX = 20;

/****** temporary data ******/
double rx_angle;
double rx_strengthX;
double rx_strengthY;
// temporary string buffer
char* tempbuff = (char*)malloc(64);

/**** expected angle ****/
double pitch_target = PITCH_BIAS;
double roll_target = ROLL_BIAS;
double yaw_target = 0;

// throttle
int throttle = 0;
int motor1 = 0;
int motor2 = 0;
int motor3 = 0;
int motor4 = 0;
/*
int pitch_out = 0;
int roll_out = 0;
int yaw_out = 0;
*/
bool unLock = false;
// declare ESCDriver as global variable
ESCDriver escDriver;

void calcMotor(int throttle, double pitchout, double rollout, double yawout){
    int motor_limit = 0.4 * throttle;

    if(throttle > 23){
        motor1 = pitchout + rollout + yawout;
        motor2 = pitchout - rollout - yawout;
        motor3 = -pitchout - rollout + yawout;
        motor4 = -pitchout + rollout - yawout;
/*
        // motor1
        if(motor1 > MOTOR_MAX || motor1 > motor_limit){
            motor1 = min(MOTOR_MAX, motor_limit);
        }
        if(motor1 < MOTOR_MIN || motor1 < -motor_limit){
            motor1 = max(MOTOR_MIN, -motor_limit);
        }

        // motor2
        if(motor2 > MOTOR_MAX || motor2 > motor_limit){
            motor2 = min(MOTOR_MAX, motor_limit);
        }
        if(motor2 < MOTOR_MIN || motor2 < -motor_limit){
            motor2 = max(MOTOR_MIN, -motor_limit);
        }

        // motor3
        if(motor3 > MOTOR_MAX || motor3 > motor_limit){
            motor3 = min(MOTOR_MAX, motor_limit);
        }
        if(motor3 < MOTOR_MIN || motor3 < -motor_limit){
            motor3 = max(MOTOR_MIN, -motor_limit);
        }

        // motor4
        if(motor4 > MOTOR_MAX || motor4 > motor_limit){
            motor4 = min(MOTOR_MAX, motor_limit);
        }
        if(motor4 < MOTOR_MIN || motor4 < -motor_limit){
            motor4 = max(MOTOR_MIN, -motor_limit);
        }
    */

        if(motor1 > MOTOR_MAX ){
            motor1 = MOTOR_MAX;
        }
        if(motor1 < MOTOR_MIN ){
            motor1 = MOTOR_MIN;
        }

        // motor2
        if(motor2 > MOTOR_MAX ){
            motor2 = MOTOR_MAX;
        }
        if(motor2 < MOTOR_MIN ){
            motor2 = MOTOR_MIN;
        }

        // motor3
        if(motor3 > MOTOR_MAX ){
            motor3 = MOTOR_MAX;
        }
        if(motor3 < MOTOR_MIN ){
            motor3 = MOTOR_MIN;
        }

        // motor4
        if(motor4 > MOTOR_MAX ){
            motor4 = MOTOR_MAX;
        }
        if(motor4 < MOTOR_MIN ){
            motor4 = MOTOR_MIN;
        }
    }
    else {
        motor1 = 0;
        motor2 = 0;
        motor3 = 0;
        motor4 = 0;
    }
}

void* parseCommand(void* command){
    int commandLength = strlen((char *) command);
    if(commandLength == 0){
        return NULL;
    }

    tempbuff = strtok((char *) command, DELIM);
    if(strcmp(tempbuff, LEFT) == 0){
        tempbuff = strtok(NULL, DELIM);
        rx_angle = atof(tempbuff);
        tempbuff = strtok(NULL, DELIM);
        rx_strengthX = atof(tempbuff);
        tempbuff = strtok(NULL, DELIM);
        rx_strengthY = atof(tempbuff);

        // calculate expected yaw
        if(rx_angle > 90 && rx_angle < 270){
            yaw_target = rx_strengthX * ANGLE_MAX;
            //yaw_out = rx_strengthX * MOTOR_MAX;
        }
        else{
            yaw_target = -(rx_strengthX * ANGLE_MAX);
           //yaw_out = rx_strengthX * MOTOR_MIN;
        }

        // calculat throttle
        throttle = (rx_strengthY * THR_BASE);

        // printf("Throttle: %x\tYaw: %lf\n", throttle, yaw_target);
    }
    else if(strcmp(tempbuff, RIGHT) == 0){
        tempbuff = strtok(NULL, DELIM);
        rx_angle = atof(tempbuff);
        tempbuff = strtok(NULL, DELIM);
        rx_strengthX = atof(tempbuff);
        tempbuff = strtok(NULL, DELIM);
        rx_strengthY = atof(tempbuff);

        // calculate expected pitch
        if(rx_angle >= 0 && rx_angle <= 180){
            pitch_target = -(rx_strengthY * ANGLE_MAX) + PITCH_BIAS;
           //pitch_out = rx_strengthY * MOTOR_MIN;
        }
        else{
            pitch_target = rx_strengthY * ANGLE_MAX + PITCH_BIAS;
            //pitch_out = rx_strengthY * MOTOR_MAX;
        }

        // calculate expected roll
        if(rx_angle > 90 && rx_angle < 270){
            roll_target = -(rx_strengthX * ANGLE_MAX) + ROLL_BIAS;
            //roll_out = rx_strengthX * MOTOR_MIN;
        }
        else{
            roll_target = rx_strengthX * ANGLE_MAX + ROLL_BIAS;
            //roll_out = rx_strengthX * MOTOR_MAX;
        }

        // printf("+Pitch: %lf\tRoll: %lf\n", pitch_target, roll_target);
    }

    else if(strcmp(tempbuff, ACTION) == 0){
        tempbuff = strtok(NULL, DELIM);
        if(strcmp(tempbuff, "unlock") == 0){
            escDriver.unLock();
            delay(3000);
            escDriver.setESCALL(ESCDriver::DC_IDLE);
            unLock = true;
        }
        else if(strcmp(tempbuff, "lock") == 0){
            escDriver.lock();
            unLock = false;
        }
    }

/*
    calcMotor(throttle,  pitch_out, roll_out, yaw_out);

    printf("Motor1: %d\n", motor1);
    printf("Motor2: %d\n", motor2);
    printf("Motor3: %d\n", motor3);
    printf("Motor4: %d\n", motor4);

    escDriver.setESCALL(THR_MIN + throttle + motor1,
                                                      THR_MIN + throttle + motor2,
                                                      THR_MIN + throttle + motor3,
                                                      THR_MIN + throttle + motor4);
*/
    return NULL;
}

/**** echo and receiver thread ****/
void* echo_thread(void* echo){
    ((Echo *) echo)->standBy();
    pthread_exit(NULL);
}

void* cntl_conn_thread(void* conn){
    ((CtrlRecv *) conn)->standByConnect();
    pthread_exit(NULL);
}

void* cntl_recv_thread(void* recv){
    ((CtrlRecv *) recv)->standByReceive(&parseCommand);
    pthread_exit(NULL);
}

/**** contrller thread ****/
void* control_thread(void* mpu){
    MPU9250* mpu9250 = (MPU9250*) mpu;

    double accel_x = 0,
            accel_y = 0,
            accel_z = 0;

    double gyro_x = 0,
            gyro_y = 0,
            gyro_z = 0;

    double pitch = 0,
            roll = 0,
            yaw = 0;
    double pitch_accel = 0,
            roll_accel = 0;

    KalmanFilter pitch_filter,
                 roll_filter,
                 yaw_filter;

    CascadePID pitch_pid,
               roll_pid,
               yaw_pid;

    // PID output
    double pitch_out = 0,
            roll_out = 0,
            yaw_out = 0;

    int lasttime,
        nowtime;

/**** Set first angle for kalman filter ****/
    lasttime = now();

    accel_x = mpu9250->readAccelX();
    accel_y = mpu9250->readAccelY();
    accel_z = mpu9250->readAccelZ();

    pitch_accel = atan(accel_x / sqrt(pow(accel_y, 2) + pow(accel_z, 2))) * (180/PI);
    roll_accel = atan(accel_y / sqrt(pow(accel_x, 2) + pow(accel_z, 2))) * (180/PI);

    pitch_filter.setAngle(pitch_accel);
    roll_filter.setAngle(roll_accel);

    while(1){
        accel_x = mpu9250->readAccelX();
        accel_y = mpu9250->readAccelY();
        accel_z = mpu9250->readAccelZ();

        nowtime = now();

        gyro_x = mpu9250->readGyroX();
        gyro_y = mpu9250->readGyroY();
        gyro_z = mpu9250->readGyroZ();

        pitch_accel = atan(accel_x / sqrt(pow(accel_y, 2) + pow(accel_z, 2))) * (180/PI);
        roll_accel = atan(accel_y / sqrt(pow(accel_x, 2) + pow(accel_z, 2))) * (180/PI);

        pitch = pitch_filter.updateAngle(pitch_accel, gyro_y, nowtime - lasttime);
        roll = roll_filter.updateAngle(roll_accel, gyro_x, nowtime - lasttime);

        lasttime = nowtime;

        // printf("Pitch: %lf\tRoll: %lf\n", pitch, roll);

        pitch_out = pitch_pid.calculate(pitch_target - pitch, -gyro_y);
        roll_out = roll_pid.calculate(roll_target - roll, gyro_x);
        yaw_out = yaw_pid.calculate(yaw_target - yaw, gyro_z);
/*
        printf("Pitch: %lf\tPitch Out: %lf\n", pitch, pitch_out);
        printf("Roll: %lf\tRoll out: %lf\n", roll, roll_out);
        printf("Yaw: %lf\tYaw out: %lf\n", yaw, yaw_out);
*/
        // If ESC is unlocked
        if(unLock){
            calcMotor(throttle, pitch_out, roll_out, yaw_out);

            printf("Motor1: %d\n", motor1);
            printf("Motor2: %d\n", motor2);
            printf("Motor3: %d\n", motor3);
            printf("Motor4: %d\n", motor4);


            escDriver.setESCALL(THR_MIN + throttle + motor1,
                                THR_MIN + throttle + motor2,
                                THR_MIN + throttle + motor3,
                                THR_MIN + throttle + motor4);
        }

        delay(20);
    }

    pthread_exit(NULL);
}

int main(){
    pthread_t echo_tid = 0;
    pthread_t conn_tid = 0;
    pthread_t recv_tid = 0;
    pthread_t pid_tid = 0;

    Echo* echo = new Echo();
    CtrlRecv* cntlRecv = new CtrlRecv();
    // Set pwm frequency to 50Hz
    escDriver.setFrequency(50);

    MPU9250* mpu9250 = new MPU9250();
    mpu9250->setupAccel(mpu9250->ACCEL_FS_2);
    mpu9250->setupGyro(mpu9250->GYRO_FS_500);

    // Echo Socket
    if(echo->bindTo() == -1){
        printf("Bind failed!\n");
        return -1;
    }
    if(echo->listenOn()){
        printf("Listen failed!\n");
        return -1;
    }

    // Connect and Receive Socket
    if(cntlRecv->bindTo() == -1){
        printf("Bind failed!\n");
        return -1;
    }

    if(cntlRecv->listenOn() == -1){
        printf("Listen failed!\n");
        return -1;
    }

    pthread_create(&echo_tid, NULL, &echo_thread, echo);
    pthread_create(&conn_tid, NULL, &cntl_conn_thread, cntlRecv);
    pthread_create(&recv_tid, NULL, &cntl_recv_thread, cntlRecv);
    pthread_create(&pid_tid, NULL, &control_thread, mpu9250);

    pthread_join(echo_tid, NULL);
    pthread_join(conn_tid, NULL);
    pthread_join(recv_tid, NULL);
    pthread_join(pid_tid, NULL);

/*
    escDriver.unLock();
    delay(3000);

    escDriver.setESCALL(THR_MIN + 12, THR_MIN , THR_MIN , THR_MIN);
    delay(3000);
    escDriver.setESCALL(THR_MIN , THR_MIN+12 , THR_MIN , THR_MIN);
    delay(3000);
    escDriver.setESCALL(THR_MIN , THR_MIN , THR_MIN+ 12 , THR_MIN);
    delay(3000);
    escDriver.setESCALL(THR_MIN , THR_MIN , THR_MIN , THR_MIN+ 12);
    delay(3000);

    escDriver.lock();
*/
    return 0;
}
