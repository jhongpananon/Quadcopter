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
         * @param[in] pitch     Pitch value from -100 -> +100
         * @param[in] roll      Roll value from -100 -> +100
         * @param[in] yaw       Yaw value from -100 -> +100
         * @parma[in] throttle  The throttle value from 0 -> +100
         *
         * TODO : This should be a hidden interface, not everyone should be allowed to do this
         */
        void setFlightParameters(int8_t pitch, int8_t roll, int8_t yaw, uint8_t throttle)
        {

        }
        /** @} */

    protected:
    private:
        ThreeAxisSensor mAccelerationSensor;    ///< Acceleration sensor data
        ThreeAxisSensor mGyroSensor;            ///< Gyroscope sensor data
        ThreeAxisSensor mMagnoSensor;           ///< Magnetometer sensor data

        int8_t mInputPitch;     ///< User input for pitch
        int8_t mInputRoll;      ///< User input for roll
        int8_t mInputYaw;       ///< User input for yaw
};



#endif /* FLIGHT_CONTROLLER_HPP_ */
