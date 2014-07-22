/*
 * sensor_system.hpp
 *
 *  Created on: Jul 13, 2014
 *      Author: pardeep
 */

#ifndef SENSOR_SYSTEM_HPP_
#define SENSOR_SYSTEM_HPP_

#include "sensor_ifaces.hpp"
#include "adafruit_10dof/bmp085.hpp"

//#include "Adafruit_BMP085_U.h"
//#include "Adafruit_L3GD20_U.h"
//#include "Adafruit_LSM303_U.h"

class SensorSystem : public iMagnoIface, public iAcceleroIface, public iGyroIface
{
    public:
        SensorSystem();
        threeAxisVector_t getAngularData(void) const  { return mAngularCachedData;  }
        threeAxisVector_t getAcceleroData(void) const { return mAcceleroCachedData; }
        threeAxisVector_t getGyroData(void) const     { return mGyroCachedData;     }
        void updateSensorData(void);

    private:
        threeAxisVector_t               mAngularCachedData;
        threeAxisVector_t               mAcceleroCachedData;
        threeAxisVector_t               mGyroCachedData;
        bmp085         mPressureSensor;
//        Adafruit_L3GD20_Unified         mGyroSensor;
//        Adafruit_LSM303_Accel_Unified   mAcceleroSensor;
//        Adafruit_LSM303_Mag_Unified     mMagnoSensor;
};

#endif /* SENSOR_SYSTEM_HPP_ */
