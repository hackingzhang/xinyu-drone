#include "ESCDriver.h"
#include <wiringPi.h>
#include <wiringPiI2C.h>

ESCDriver::ESCDriver()
{
    this->i2cHandler = wiringPiI2CSetup(i2cAddress);
    setModeSleep();
}

void ESCDriver::setModeNormal(){
    wiringPiI2CWriteReg8(i2cHandler, MODE1_REG_ADDR, 0x01);
}

void ESCDriver::setModeSleep(){
    wiringPiI2CWriteReg8(i2cHandler, MODE1_REG_ADDR, 0x10);
}

void ESCDriver::setFrequency(int frequency){
    // prescale = (osc_clock / (4096 * frequency)) - 1
    int data = 25000000 /(4096 * frequency) - 1;

    wiringPiI2CWriteReg8(i2cHandler, PRESCALE_REG_ADDR, data);
}

void ESCDriver::lock(){
    setModeSleep();
}

void ESCDriver::unLock(){
    wiringPiI2CWriteReg8(i2cHandler, ESCALL_ON_ADDR_L, getLow(DELAY_TIME));
    wiringPiI2CWriteReg8(i2cHandler, ESCALL_ON_ADDR_H, getHigh(DELAY_TIME));
    wiringPiI2CWriteReg8(i2cHandler, ESCALL_OFF_ADDR_L, getLow(DC_MIN));
    wiringPiI2CWriteReg8(i2cHandler, ESCALL_OFF_ADDR_H, getHigh(DC_MIN));

    setModeNormal();
}

void ESCDriver::setESC1(int dc){
    wiringPiI2CWriteReg8(i2cHandler, ESCALL_OFF_ADDR_L, getLow(dc));
    wiringPiI2CWriteReg8(i2cHandler, ESCALL_OFF_ADDR_H, getHigh(dc));
}

void ESCDriver::setESC2(int dc){
    wiringPiI2CWriteReg8(i2cHandler, ESCALL_OFF_ADDR_L, getLow(dc));
    wiringPiI2CWriteReg8(i2cHandler, ESCALL_OFF_ADDR_H, getHigh(dc));
}

void ESCDriver::setESC3(int dc){
    wiringPiI2CWriteReg8(i2cHandler, ESCALL_OFF_ADDR_L, getLow(dc));
    wiringPiI2CWriteReg8(i2cHandler, ESCALL_OFF_ADDR_H, getHigh(dc));
}

void ESCDriver::setESC4(int dc){
    wiringPiI2CWriteReg8(i2cHandler, ESCALL_OFF_ADDR_L, getLow(dc));
    wiringPiI2CWriteReg8(i2cHandler, ESCALL_OFF_ADDR_H, getHigh(dc));
}

void ESCDriver::setESCALL(int dc){
    wiringPiI2CWriteReg8(i2cHandler, ESCALL_OFF_ADDR_L, getLow(dc));
    wiringPiI2CWriteReg8(i2cHandler, ESCALL_OFF_ADDR_H, getHigh(dc));
}

void ESCDriver::setESCALL(int esc1, int esc2, int esc3, int esc4){
    setESC1(esc1);
    setESC2(esc2);
    setESC3(esc3);
    setESC4(esc4);
}

ESCDriver::~ESCDriver()
{
    //dtor
    setModeSleep();
}
