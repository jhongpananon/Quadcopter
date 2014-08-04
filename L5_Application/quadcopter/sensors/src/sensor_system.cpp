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
    mI2C.writeReg(I2CAddr_LSM303_Accel, 0x20, 0x77);    /// Enable the 3-axis and set update rate to 400hz
    mI2C.writeReg(I2CAddr_LSM303_Accel, 0x23, 0x00);    /// Disable BLE at CTRL_REG4 and set to +/- 2G

    /* Initialize the magnometer sensor */
    bool m = mI2C.checkDeviceResponse(I2CAddr_LSM303_Mag);
    mI2C.writeReg(I2CAddr_LSM303_Mag, 0x00, 0x1C);  /// Set update rate to 220Hz
    mI2C.writeReg(I2CAddr_LSM303_Mag, 0x01, 0x02);  /// Set gain to +/- 1.3 gauss
    mI2C.writeReg(I2CAddr_LSM303_Mag, 0x02, 0x00);  /// Continuous conversion mode

    /* Initialize the gyroscope */
    bool g = mI2C.checkDeviceResponse(I2CAddr_L3GD20_Gyro);
    mI2C.writeReg(I2CAddr_L3GD20_Gyro, 0x20, 0x00);          /// Reset control register
    mI2C.writeReg(I2CAddr_L3GD20_Gyro, 0x20, (0xB0 | 0x0F)); /// Enable sensor, the 3 axis, and use 380Hz update rate
    mI2C.writeReg(I2CAddr_L3GD20_Gyro, 0x23, 0x00);          /// Disable BLE at CTRL_REG4

    /* Struct size should be six bytes, and all sensor should have responded */
    return (6 == sizeof(rawSensorVector_t) && a && m && g);
}

void SensorSystem::updateSensorData()
{
    rawSensorVector_t raw;
    const uint8_t readMultiBytes = 0x80; ///< Bit7 must be set to perform address auto-increment on accelero and gyro
    const float degPerBitFor250dps = 8.75 / 1000.0f;    ///< Datasheet: 8.75 mdps for 250 deg/sec
    const float degPerSecToRadPerSec = 0.0174532925f;   ///< Standard conversion formula

    mI2C.readRegisters(I2CAddr_LSM303_Accel, 0x28 | readMultiBytes, (char *)&raw, sizeof(raw));

    /* Can't use direct conversion unless we can read MSB byte first like magno sensor */
#if 0
    mAcceleroCachedData.x = (raw.x) >> 4;
    mAcceleroCachedData.y = (raw.y) >> 4;
    mAcceleroCachedData.z = (raw.z) >> 4;
#else
    mAcceleroCachedData.x = (int16_t)((raw.xl | ((uint16_t)raw.xh << 8))) >> 4;
    mAcceleroCachedData.y = (int16_t)((raw.yl | ((uint16_t)raw.yh << 8))) >> 4;
    mAcceleroCachedData.z = (int16_t)((raw.zl | ((uint16_t)raw.zh << 8))) >> 4;
#endif

    /* Magno data doesn't need conversion as our raw data is read in the correct
     * byte-order, which is high byte first, then low byte.
     */
    mI2C.readRegisters(I2CAddr_LSM303_Mag, 0x03, (char *)&raw, sizeof(raw));
    mMagnoCachedData.x = raw.x;
    mMagnoCachedData.y = raw.y;
    mMagnoCachedData.z = raw.z;

    /* Read the gyroscope and convert the readings to radians per second that is needed by AHRS algorithm */
    mI2C.readRegisters(I2CAddr_L3GD20_Gyro, 0x28 | readMultiBytes, (char *)&raw, sizeof(raw));
#if 0
    mGyroAngularCachedData.x = raw.x;
    mGyroAngularCachedData.y = raw.y;
    mGyroAngularCachedData.z = raw.z;
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
}
