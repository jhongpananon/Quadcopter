/*
 * sensor_system.cpp
 *
 *  Created on: Jul 13, 2014
 *      Author: pardeep
 */
#include <stdio.h>

#include "sensor_system.hpp"



SensorSystem::SensorSystem() : mI2C(I2C2::getInstance())
{

}

bool SensorSystem::init(void)
{
    /* Initialize the acceleration sensor */
    bool a = mI2C.checkDeviceResponse(I2CAddr_LSM303_Accel);
    mI2C.writeReg(I2CAddr_LSM303_Accel, 0x20, 0x57);
    mI2C.writeReg(I2CAddr_LSM303_Accel, 0x23, 0x00);

    /* Initialize the magnometer sensor */
//    mI2C.writeReg(I2CAddr_LSM303_Mag, 0x00, 0x14);
//    mI2C.writeReg(I2CAddr_LSM303_Mag, 0x01, 0x80);
    bool m = mI2C.checkDeviceResponse(I2CAddr_LSM303_Mag);
    mI2C.writeReg(I2CAddr_LSM303_Mag, 0x02, 0);

    /* Initialize the gyroscope */
    bool g = mI2C.checkDeviceResponse(I2CAddr_L3GD20_Gyro);
    mI2C.writeReg(I2CAddr_L3GD20_Gyro, 0x20, 0);
    mI2C.writeReg(I2CAddr_L3GD20_Gyro, 0x20, 0x0F);
    mI2C.writeReg(I2CAddr_L3GD20_Gyro, 0x23, 0x00);

    /* Struct size should be six bytes, and all sensor should have responded */
    return (6 == sizeof(rawSensorVector_t) && a && m && g);
}

void SensorSystem::updateSensorData()
{
    rawSensorVector_t raw;
    const uint8_t readMultiBytes = 0x80;
    const float degPerBitFor250dps = 8.75 / 1000.0f;    ///< 8.75 mdps for 250 deg/sec
    const float degPerSecToRadPerSec = 0.0174532925f;   ///< Standard conversion formula

    mI2C.readRegisters(I2CAddr_LSM303_Accel, 0x28 | readMultiBytes, (char *)&raw, sizeof(raw));

    /* Can't use direct conversion unless we can read MSB byte first like magno sensor */
#if 0
    mAcceleroCachedData.x = (int16_t)(raw.x) >> 4;
    mAcceleroCachedData.y = (int16_t)(raw.y) >> 4;
    mAcceleroCachedData.z = (int16_t)(raw.z) >> 4;
#else
    mAcceleroCachedData.x = (int16_t)((raw.xl | ((uint16_t)raw.xh << 8))) >> 4;
    mAcceleroCachedData.y = (int16_t)((raw.yl | ((uint16_t)raw.yh << 8))) >> 4;
    mAcceleroCachedData.z = (int16_t)((raw.zl | ((uint16_t)raw.zh << 8))) >> 4;
#endif

//    mAcceleroCachedData.x *= 0.001F * 9.80665F;
//    mAcceleroCachedData.y *= 0.001F * 9.80665F;
//    mAcceleroCachedData.z *= 0.001F * 9.80665F;
//    printf("Accel = %5.1f, %5.1f, %5.1f\n", mAcceleroCachedData.x, mAcceleroCachedData.y, mAcceleroCachedData.z);

    /* Magno data doesn't need conversion as our raw data is read in the correct
     * byte-order, which is high byte first, then low byte.
     */
    mI2C.readRegisters(I2CAddr_LSM303_Mag, 0x03, (char *)&raw, sizeof(raw));
    mMagnoCachedData.x = raw.x;
    mMagnoCachedData.y = raw.y;
    mMagnoCachedData.z = raw.z;
    //printf("Magno = %5.1f, %5.1f, %5.1f\n", mMagnoCachedData.x, mMagnoCachedData.y, mMagnoCachedData.z);

    mI2C.readRegisters(I2CAddr_L3GD20_Gyro, 0x28 | readMultiBytes, (char *)&raw, sizeof(raw));
#if 0
    mGyroAngularCachedData.x = (int16_t)(raw.x);
    mGyroAngularCachedData.y = (int16_t)(raw.y);
    mGyroAngularCachedData.z = (int16_t)(raw.z);
#else
    mGyroAngularCachedData.x = (int16_t)(raw.xl | ((uint16_t)raw.xh << 8));
    mGyroAngularCachedData.y = (int16_t)(raw.yl | ((uint16_t)raw.yh << 8));
    mGyroAngularCachedData.z = (int16_t)(raw.zl | ((uint16_t)raw.zh << 8));
#endif

    /* Avoid multiplying (degPerBitFor250dps * degPerSecToRadPerSec) since this calculation
     * is too closer to zero value of 32-bit single precision float
     */
    mGyroAngularCachedData.x *= degPerBitFor250dps;
    mGyroAngularCachedData.y *= degPerBitFor250dps;
    mGyroAngularCachedData.z *= degPerBitFor250dps;
    mGyroAngularCachedData.x *= degPerSecToRadPerSec;
    mGyroAngularCachedData.y *= degPerSecToRadPerSec;
    mGyroAngularCachedData.z *= degPerSecToRadPerSec;
//    printf("Gyro = %5.1f, %5.1f, %5.1f\n", mGyroAngularCachedData.x, mGyroAngularCachedData.y, mGyroAngularCachedData.z);
}
