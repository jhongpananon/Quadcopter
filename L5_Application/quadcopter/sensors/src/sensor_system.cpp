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
    /* Equal operator so much easier than memset() to zero :)  */
    mRawAccelero = 0;
    mRawMagno = 0;
    mRawGyro = 0;

    mCalAccelero = 0;
    mCalMagno = 0;
    mCalGyro = 0;

    mAcceleroCachedData = 0;
    mMagnoCachedData = 0;
    mGyroAngularCachedData = 0;
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
    mI2C.writeReg(I2CAddr_LSM303_Mag, 0x01, 0x20);  /// Set gain to +/- 1.3 gauss
    mI2C.writeReg(I2CAddr_LSM303_Mag, 0x02, 0x00);  /// Continuous conversion mode

    /* Initialize the gyroscope */
    bool g = mI2C.checkDeviceResponse(I2CAddr_L3GD20_Gyro);
    mI2C.writeReg(I2CAddr_L3GD20_Gyro, 0x20, 0x00);          /// Reset control register
    mI2C.writeReg(I2CAddr_L3GD20_Gyro, 0x20, (0xB0 | 0x0F)); /// Enable sensor, the 3 axis, and use 380Hz update rate
    mI2C.writeReg(I2CAddr_L3GD20_Gyro, 0x23, 0x00);          /// Disable BLE at CTRL_REG4

    /* Struct size should be six bytes, and all sensor should have responded */
    return (6 == sizeof(rawSensorVector_t) && a && m && g);
}

void SensorSystem::updateSensorData(void)
{
    const uint8_t readMultiBytes = 0x80; ///< Bit7 must be set to perform address auto-increment on accelero and gyro

    /* Read raw data and convert 16-bit data to 12-bit data */
    mI2C.readRegisters(I2CAddr_LSM303_Accel, 0x28 | readMultiBytes, (char*)&mRawAccelero, sizeof(mRawAccelero));
    mAcceleroCachedData = convertRawAccelero(mRawAccelero);

    /* Read raw data and store it, no conversion needed since magno data just needs a vector without units
     * We need to byte swap since magno registers are HIGH first then LOW
     */
    mI2C.readRegisters(I2CAddr_LSM303_Mag, 0x03, (char*)&mRawMagno, sizeof(mRawMagno));
    mMagnoCachedData = convertRawMagno(mRawMagno);

    /* Read the gyroscope and convert the readings to radians per second that is needed by AHRS algorithm */
    mI2C.readRegisters(I2CAddr_L3GD20_Gyro, 0x28 | readMultiBytes, (char*)&mRawGyro, sizeof(mRawGyro));
    mGyroAngularCachedData = convertRawGyro(mRawGyro);
}


threeAxisVector_t SensorSystem::convertRawAccelero(rawSensorVector_t &raw)
{
    threeAxisVector_t f;

    /* Adafruit is doing this, and I've noticed that the smallest change is 32 at +/- 2G */
    raw.x >>= 4;
    raw.y >>= 4;
    raw.z >>= 4;

    /* Apply calibration values */
    raw += mCalAccelero;

    /* Store to float */
    f.x = raw.x;
    f.y = raw.y;
    f.z = raw.z;

    return f;
}

threeAxisVector_t SensorSystem::convertRawGyro(rawSensorVector_t &raw)
{
    threeAxisVector_t f;
    const float degPerBitFor250dps = 8.75 / 1000.0f;    ///< Datasheet: 8.75 mdps for 250 deg/sec
    const float degPerSecToRadPerSec = 0.0174532925f;   ///< Standard conversion formula

    /* Apply calibration values */
    raw += mCalGyro;

    /* Store to float */
    f.x = raw.x;
    f.y = raw.y;
    f.z = raw.z;

    /* Avoid multiplying (degPerBitFor250dps * degPerSecToRadPerSec) since this calculation
     * is too close to zero value of 32-bit single precision float
     */
    f.x *= degPerBitFor250dps;
    f.y *= degPerBitFor250dps;
    f.z *= degPerBitFor250dps;

    f.x *= degPerSecToRadPerSec;
    f.y *= degPerSecToRadPerSec;
    f.z *= degPerSecToRadPerSec;

    return f;
}

threeAxisVector_t SensorSystem::convertRawMagno(rawSensorVector_t &raw)
{
    threeAxisVector_t f;

    /* Magno requires byte swap since HIGH byte register is read first before LOW byte */
    rawSensorVectorByteSwap(raw);

    /* Apply calibration values */
    raw += mCalMagno;

    /* Store to float
     * Datasheet says: Range should be 0xF800 -> 0x07FF (-2048 -> +2047)
     */
    f.x = raw.x;
    f.y = raw.y;
    f.z = raw.z;

    return f;
}

void SensorSystem::rawSensorVectorByteSwap(rawSensorVector_t &raw)
{
    uint8_t high;
    high = raw.xh;  raw.xh = raw.xl; raw.xl = high;
    high = raw.yh;  raw.yh = raw.yl; raw.yl = high;
    high = raw.zh;  raw.zh = raw.zl; raw.zl = high;
}

bool SensorSystem::calibrate(void)
{
    /* Need a large signed int to add up the sum of all samples */
    int64_t x = 0, y = 0, z = 0;
    const int32_t samples = 100;

    /* Important to zero out calibration values because updateSensorData() gives us
     * readings after applying the calibration
     */
    mCalAccelero = 0;
    mCalMagno = 0;
    mCalGyro = 0;

    /* Find the average zero offset when the sensor is at rest */
    for (int32_t i = 0; i < samples; i++)
    {
        updateSensorData();
        x += getRawAcceleroData().x;
        y += getRawAcceleroData().y;
        z += getRawAcceleroData().z;
    }

    x /= samples;
    y /= samples;
    z /= samples;

    /* X and Y axis should be zero with the sensor flat, and at rest */
    mCalAccelero.x = -x;
    mCalAccelero.y = -y;

    /* Y should be equivalent to the full gravity pull */
    // mCalAccelero.z = z;

    return true;
}
