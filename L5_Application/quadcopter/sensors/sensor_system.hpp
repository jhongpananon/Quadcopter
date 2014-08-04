/*
 * sensor_system.hpp
 *
 *  Created on: Jul 13, 2014
 *      Author: pardeep
 */

#ifndef SENSOR_SYSTEM_HPP_
#define SENSOR_SYSTEM_HPP_

#include <stdint.h>

#include "i2c2.hpp"
#include "sensor_ifaces.hpp"



/**
 * The sensor system class that communicates with the sensors.
 * This inherits the interfaces that provide readings to the sensors to the flight stabilizer class.
 */
class SensorSystem : public iMagnoIface, public iAcceleroIface, public iGyroIface
{
    public:
        SensorSystem();

        /// Initializes the sensors and returns true if successful
        bool init(void);

        /// @returns interface method that returns the magnetometer readings
        threeAxisVector_t getMagnoData(void) const          { return mMagnoCachedData;      }

        /// @returns interface method that returns the accelerometer readings
        threeAxisVector_t getAcceleroData(void) const       { return mAcceleroCachedData;   }

        /// @returns interface method that returns the gyroscope readings
        threeAxisVector_t getGyroAngularData(void) const    { return mGyroAngularCachedData;}

        /// Updates all the sensor data to be later retrieved by the interface methods
        void updateSensorData(void);

    private:
        /// The data structure read from the magno, accelero, and the gyro registers
        typedef union
        {
            struct {
                uint8_t xl, xh;     ///< X-axis low and high byte
                uint8_t yl, yh;     ///< Y-axis low and high byte
                uint8_t zl, zh;     ///< Z-axis low and high byte
            };

            struct {
                int16_t x, y, z;    ///< Overlapping memory of high/low bytes into a single 16-bit 2's compliment int
            };
        } __attribute__ ((packed)) rawSensorVector_t;

        threeAxisVector_t mMagnoCachedData;         ///< updateSensorData() caches the magno readings here
        threeAxisVector_t mAcceleroCachedData;      ///< updateSensorData() caches the accelero readings here
        threeAxisVector_t mGyroAngularCachedData;   ///< updateSensorData() caches the gyro readings here
        I2C2              &mI2C;    ///< The instance of I2C bus that is used to communicate with the sensors
};

#endif /* SENSOR_SYSTEM_HPP_ */
