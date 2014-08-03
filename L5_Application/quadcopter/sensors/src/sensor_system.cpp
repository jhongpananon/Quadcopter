/*
 * sensor_system.cpp
 *
 *  Created on: Jul 13, 2014
 *      Author: pardeep
 */

#include "sensor_system.hpp"
#include <stdio.h>

SensorSystem::SensorSystem() : mI2C(I2C2::getInstance())
{

}

bool SensorSystem::init(void)
{
    /*
     * Initialize the acceleration sensor:
     */
    mI2C.writeReg(I2CAddr_LSM303_Accel, 0x20, 0x57);
    mI2C.writeReg(I2CAddr_LSM303_Accel, 0x23, 0x00);

    /*
     * Initialize the magnometer sensor:
     */
//    mI2C.writeReg(I2CAddr_LSM303_Mag, 0x00, 0x14);
//    mI2C.writeReg(I2CAddr_LSM303_Mag, 0x01, 0x80);
    mI2C.writeReg(I2CAddr_LSM303_Mag, 0x02, 0);

    /*
     * Initialize the gyroscope
     */
    mI2C.writeReg(I2CAddr_L3GD20_Gyro, 0x20, 0);
    mI2C.writeReg(I2CAddr_L3GD20_Gyro, 0x20, 0x0F);
    mI2C.writeReg(I2CAddr_L3GD20_Gyro, 0x23, 0x00);

    return (6 == sizeof(rawSensorVector_t));
}

void SensorSystem::updateSensorData()
{
    rawSensorVector_t   raw;

    mI2C.readRegisters(I2CAddr_LSM303_Accel, 0x28 | 0x80, (char *)&raw, sizeof(raw));
//    mAcceleroCachedData.x = (int16_t)(raw.x) >> 4;
//    mAcceleroCachedData.y = (int16_t)(raw.y) >> 4;
//    mAcceleroCachedData.z = (int16_t)(raw.z) >> 4;

    mAcceleroCachedData.x = (int16_t)((raw.xl | ((uint16_t)raw.xh << 8))) >> 4;
    mAcceleroCachedData.y = (int16_t)((raw.yl | ((uint16_t)raw.yh << 8))) >> 4;
    mAcceleroCachedData.z = (int16_t)((raw.zl | ((uint16_t)raw.zh << 8))) >> 4;

//    mAcceleroCachedData.x *= 0.001F * 9.80665F;
//    mAcceleroCachedData.y *= 0.001F * 9.80665F;
//    mAcceleroCachedData.z *= 0.001F * 9.80665F;
//    printf("Accel = %5.1f, %5.1f, %5.1f\n", mAcceleroCachedData.x, mAcceleroCachedData.y, mAcceleroCachedData.z);

    mI2C.readRegisters(I2CAddr_LSM303_Mag, 0x03, (char *)&raw, sizeof(raw));
    mMagnoCachedData.x = raw.x;
    mMagnoCachedData.y = raw.y;
    mMagnoCachedData.z = raw.z;
    //printf("Magno = %5.1f, %5.1f, %5.1f\n", mMagnoCachedData.x, mMagnoCachedData.y, mMagnoCachedData.z);

    mI2C.readRegisters(I2CAddr_L3GD20_Gyro, 0x28 | 0x80, (char *)&raw, sizeof(raw));
//    mGyroAngularCachedData.x = (int16_t)(raw.x);
//    mGyroAngularCachedData.y = (int16_t)(raw.y);
//    mGyroAngularCachedData.z = (int16_t)(raw.z);
    mGyroAngularCachedData.x = (int16_t)(raw.xl | ((uint16_t)raw.xh << 8));
    mGyroAngularCachedData.y = (int16_t)(raw.yl | ((uint16_t)raw.yh << 8));
    mGyroAngularCachedData.z = (int16_t)(raw.zl | ((uint16_t)raw.zh << 8));
    mGyroAngularCachedData.x /= 250.0f;
    mGyroAngularCachedData.y /= 250.0f;
    mGyroAngularCachedData.z /= 250.0f;
//    printf("Gyro = %5.1f, %5.1f, %5.1f\n", mGyroAngularCachedData.x, mGyroAngularCachedData.y, mGyroAngularCachedData.z);
}
