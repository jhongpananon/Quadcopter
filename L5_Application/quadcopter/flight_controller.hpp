/**
 * @file
 */
#ifndef FLIGHT_CONTROLLER_HPP_
#define FLIGHT_CONTROLLER_HPP_
#include <stdint.h>
#include "three_axis_sensor.hpp"



/**
 * Common structure used for pitch, roll, yaw, and throttle values
 */
typedef struct {
    int8_t pitch;     ///< Pitch value
    int8_t roll;      ///< Roll value
    int8_t yaw;       ///< Yaw value
    uint8_t throttle; ///< Throttle value
} flight_params_t;

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

        /**
         * Runs a filter on the raw inputs supplied to the sensors
         * XXX: Maybe a first stage "smoothing" filter here?
         *      This should call filters at ThreeAxisSensor class
         */
        void runRawInputFilter(void);

        void runPID(void);
        void runKalmanFilter(void);

        /**
         * @{ API to set flight parameters
         *
         * @param [in] params   The flight parameters with these members:
         *                          - pitch     Pitch value from -100 -> +100
         *                          - roll      Roll value from -100 -> +100
         *                          - yaw       Yaw value from -100 -> +100
         *                          - throttle  The throttle value from 0 -> +100
         *
         * TODO : This should be a hidden interface, not everyone should be allowed to do this
         * because only Quadcopter should set the input parameters based on its logic.
         */
        void setFlightParameters(const flight_params_t& params)
        {
            mInputFlightParams = params;
        }
        /** @} */

    protected:
    private:
        ThreeAxisSensor mAccelerationSensor;    ///< Acceleration sensor data
        ThreeAxisSensor mGyroSensor;            ///< Gyroscope sensor data
        ThreeAxisSensor mMagnoSensor;           ///< Magnetometer sensor data
        flight_params_t mInputFlightParams;     ///< Input flight parameters
};



#endif /* FLIGHT_CONTROLLER_HPP_ */
