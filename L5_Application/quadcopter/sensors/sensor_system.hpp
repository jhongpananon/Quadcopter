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


class SensorSystem : public iMagnoIface, public iAcceleroIface, public iGyroIface
{
    public:
        SensorSystem();
        bool init(void);
        threeAxisVector_t getMagnoData(void) const           { return mMagnoCachedData;  }
        threeAxisVector_t getAcceleroData(void) const        { return mAcceleroCachedData; }
        threeAxisVector_t getGyroAngularData(void) const     { return mGyroAngularCachedData;     }
        void updateSensorData(void);

    private:
        typedef union
        {
                struct
                {
                        uint8_t xl;
                        uint8_t xh;
                        uint8_t yl;
                        uint8_t yh;
                        uint8_t zl;
                        uint8_t zh;
                };
                struct
                {
                        int16_t x, y, z;
                };
        } __attribute__ ((packed)) rawSensorVector_t;

        threeAxisVector_t mMagnoCachedData;
        threeAxisVector_t mAcceleroCachedData;
        threeAxisVector_t mGyroAngularCachedData;
        I2C2              &mI2C;
};

#endif /* SENSOR_SYSTEM_HPP_ */
