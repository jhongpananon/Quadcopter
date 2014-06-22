/**
 * @file
 */
#ifndef FLIGHT_CONTROLLER_HPP_
#define FLIGHT_CONTROLLER_HPP_
#include <stdint.h>
#include "three_axis_sensor.hpp"



/**
 * The motor controller interface
 */
class MotorControllerIface
{
    public:
        virtual ~MotorControllerIface() { }

        /**
         * @{ Interface methods
         * These should set the PWM percentage value of the motor controllers
         */
        virtual void setNorthMotor(uint8_t throttleValuePercent) = 0;
        virtual void setSouthMotor(uint8_t throttleValuePercent) = 0;
        virtual void setEastMotor (uint8_t throttleValuePercent) = 0;
        virtual void setWestMotor (uint8_t throttleValuePercent) = 0;
        /** @} */
};

/**
 * This is the flight controller class.
 * This allows the user to set the raw sensor values, process them, and apply
 * the user input to be able to fly the Quadcopter.
 */
class FlightController : public MotorControllerIface
{
    /* Public types */
    public:
        /**
         * Common structure used for pitch, roll, yaw, and throttle values
         */
        typedef struct {
            int8_t pitch;     ///< Pitch value
            int8_t roll;      ///< Roll value
            int8_t yaw;       ///< Yaw value
            uint8_t throttle; ///< Throttle value
        } flightParams_t;

    public:
        /**
         * @{ Public sensors of the flight controller
         */
        ThreeAxisSensor mAccelerationSensor;    ///< Acceleration sensor data
        ThreeAxisSensor mGyroSensor;            ///< Gyroscope sensor data
        ThreeAxisSensor mMagnoSensor;           ///< Magnetometer sensor data
        /** @} */

        /**
         * Runs filters on the sensor inputs
         */
        void runSensorInputFilters(void)
        {

        }

    protected:
        /// Protected constructor of this abstract class
        FlightController()
        {

        }

        /**
         * @{ API to set flight parameters
         *
         * @param [in] params   The flight parameters with these members:
         *                          - pitch     Pitch value from -100 -> +100
         *                          - roll      Roll value from -100 -> +100
         *                          - yaw       Yaw value from -100 -> +100
         *                          - throttle  The throttle value from 0 -> +100
         *
         */
        void setFlightParameters(const flightParams_t& params)
        {
            mInputFlightParams = params;
        }
        /** @} */

    private:
        flightParams_t mInputFlightParams;     ///< Input flight parameters
};



#endif /* FLIGHT_CONTROLLER_HPP_ */
