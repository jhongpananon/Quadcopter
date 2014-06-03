/**
 * @file
 */
#ifndef FLIGHT_CONTROLLER_HPP_
#define FLIGHT_CONTROLLER_HPP_
#include <stdint.h>
#include "three_axis_sensor.hpp"



/**
 * This is the flight controller class.
 * This allows the user to set the raw sensor values, process them, and apply
 * the user input to be able to fly the Quadcopter.
 */
class FlightController
{
    public:
        /**
         * @{ API to set the raw sensor data values
         */
        void setRawAcceleration(const ThreeAxisSensor& data) { mAccelerationSensor = data; }
        void setRawGyro(const ThreeAxisSensor& data)         { mGyroSensor = data; }
        void setRawMagno(const ThreeAxisSensor& data)        { mMagnoSensor = data; }
        /** @} */

        void runPID(void);
        void runKalman(void);

        /**
         * @{ API to set flight parameters
         * TODO : This should be a hidden interface, not everyone should be allowed to do this
         */
        void setFlightParameters(uint32_t pitch, uint32_t roll, uint32_t yaw);
        void setLift(uint32_t percentage);
        /** @} */

    protected:
    private:
        ThreeAxisSensor mAccelerationSensor;    ///< Acceleration sensor data
        ThreeAxisSensor mGyroSensor;            ///< Gyroscope sensor data
        ThreeAxisSensor mMagnoSensor;           ///< Magnetometer sensor data
};



#endif /* FLIGHT_CONTROLLER_HPP_ */
