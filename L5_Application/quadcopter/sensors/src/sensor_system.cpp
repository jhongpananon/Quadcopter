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
    mAccelero = 0;
    mMagno = 0;
    mGyro = 0;
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
    mI2C.writeReg(I2CAddr_LSM303_Mag, 0x01, 0x60);  /// Set gain to +/- 2.5 gauss (must change convertRawMagno() if this changes)
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

    /* Read and update accelero data */
    mI2C.readRegisters(I2CAddr_LSM303_Accel, 0x28 | readMultiBytes, (char*)&mAccelero.raw, sizeof(mAccelero.raw));
    convertRawAccelero(mAccelero);

    /* Read and update magno data */
    mI2C.readRegisters(I2CAddr_LSM303_Mag, 0x03, (char*)&mMagno.raw, sizeof(mMagno.raw));
    convertRawMagno(mMagno);

    /* Read the gyroscope and convert the readings to radians per second that is needed by AHRS algorithm */
    mI2C.readRegisters(I2CAddr_L3GD20_Gyro, 0x28 | readMultiBytes, (char*)&mGyro.raw, sizeof(mGyro.raw));
    convertRawGyro(mGyro);
}


void SensorSystem::convertRawAccelero(sensorData_t &data)
{
    /* Adafruit is doing this, and I've noticed that the smallest change is 32 at +/- 2G
     * It looks like actual data resolution is 12-bit, not 16-bit as stated in the datasheet.
     */
    data.raw.x >>= 4;
    data.raw.y >>= 4;
    data.raw.z >>= 4;

    /* Apply calibration values */
    data.raw += data.offset;

    /* Store the end result */
    data.converted.x = data.raw.x;
    data.converted.y = data.raw.y;
    data.converted.z = data.raw.z;
}

void SensorSystem::convertRawGyro(sensorData_t &data)
{
    const float degPerBitFor250dps = 8.75 / 1000.0f;    ///< Datasheet: 8.75 mdps for 250 deg/sec
    const float degPerSecToRadPerSec = 0.0174532925f;   ///< Standard conversion formula

    /* Apply calibration values */
    data.raw += data.offset;

    /* Store the result in floats before we convert to radians per second */
    data.converted.x = data.raw.x;
    data.converted.y = data.raw.y;
    data.converted.z = data.raw.z;

    /* Avoid multiplying (degPerBitFor250dps * degPerSecToRadPerSec) since this calculation
     * is too close to zero value of 32-bit single precision float.
     * TODO Try dividing by 1000 last to ensure the units don't become zero.
     */
    data.converted.x *= degPerBitFor250dps;
    data.converted.y *= degPerBitFor250dps;
    data.converted.z *= degPerBitFor250dps;

    data.converted.x *= degPerSecToRadPerSec;
    data.converted.y *= degPerSecToRadPerSec;
    data.converted.z *= degPerSecToRadPerSec;
}

void SensorSystem::convertRawMagno(sensorData_t &data)
{
    /* These are the units at +/- 2.5 gauss */
    const float xyAxisLsbPerGauss = 670.0f;
    const float zAxisLsbPerGauss = 600.0f;

    /* Magno requires byte swap since HIGH byte register is read first before LOW byte */
    data.raw.byteSwap();

    /* Apply calibration values */
    data.raw += data.offset;

    /* Store to float
     * Datasheet says: Range should be 0xF800 -> 0x07FF (-2048 -> +2047)
     */
    data.converted.x = data.raw.x;
    data.converted.y = data.raw.y;
    data.converted.z = data.raw.z;

    /* Convert to the gauss units to normalize the data of this vector */
    data.converted.x /= xyAxisLsbPerGauss;
    data.converted.y /= xyAxisLsbPerGauss;
    data.converted.z /= zAxisLsbPerGauss;
}

bool SensorSystem::calibrate(void)
{
    /* Need a large signed int to add up the sum of all samples */
    int64_t x = 0, y = 0, z = 0;
    const int32_t samples = 100;

    /* Important to zero out calibration values because updateSensorData() gives us
     * readings after applying the calibration
     */
    mAccelero.offset = 0;

    /* Find the average zero offset when the sensor is at rest */
    for (int32_t i = 0; i < samples; i++)
    {
        updateSensorData();
        x += mAccelero.raw.x;
        y += mAccelero.raw.y;
        z += mAccelero.raw.z;
    }

    x /= samples;
    y /= samples;
    z /= samples;

    /* X and Y axis should be zero with the sensor flat, and at rest */
    mAccelero.offset.x = -x;
    mAccelero.offset.y = -y;

    /* Y should be equivalent to the full gravity pull */
    // mAccelero.offset.z = z;

    return true;
}
