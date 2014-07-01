/**
 * @file
 */
#ifndef FLIGHT_CONTROLLER_HPP_
#define FLIGHT_CONTROLLER_HPP_

#include <stdint.h>

#include "pid.hpp"
#include "three_axis_sensor.hpp"
#include "friend_for_tlm_reg.hpp"



/**
 * The motor controller interface
 * This is an abstract class that FlightController calls to set the propeller motor throttle values.
 */
class MotorControllerIface
{
    public:
        /// The structure that contains the motor values of each axis
        typedef struct {
            float north;    ///< North motor
            float south;    ///< South motor
            float east;     ///< East motor
            float west;     ///< West motor
        } motorValues_t;

        /// @returns the current motor values stored at this class
        inline motorValues_t getMotorValues(void) const { return mMotorValues; }

    protected:
        /// Virtual destructor of this abstract class
        virtual ~MotorControllerIface() { }

        /**
         * Interface method
         * This should set the PWM percentage value of the actual motor controllers
         * @param values    The motor values; @see motorValues_t
         */
        virtual void applyMotorValues(const motorValues_t& values) = 0;

        /**
         * Saves the motor values at this class, but doesn't apply them to the motors
         * @param values    The motor values; @see motorValues_t
         */
        inline void saveMotorValues(const motorValues_t& values) { mMotorValues = values; }

    private:
        motorValues_t mMotorValues; ///< The motor values

        // Allow private member access to register variables' telemetry
        ALLOW_FRIEND_TO_REGISTER_TLM();
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
        /// Structure of pitch, roll, and yaw
        typedef struct {
            int16_t pitch;   ///< Pitch angle
            int16_t roll;    ///< Roll angle
            int16_t yaw;     ///< Yaw angle
        } flightYPR_t;

        /// Common structure used for pitch, roll, yaw, and throttle values
        typedef struct {
            flightYPR_t angle;  /// Pitch, roll, and yaw angles
            uint8_t throttle;   ///< Throttle value
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
         * @{ API for the PID parameters
         */
        inline void setPitchAxisPidParameters(const PID::pidParams_t& params) { mPitchPid.setPidParameters(params); }
        inline void setRollAxisPidParameters(const PID::pidParams_t& params)  { mRollPid.setPidParameters(params);  }
        inline void setYawAxisPidParameters(const PID::pidParams_t& params)   { mYawPid.setPidParameters(params);   }

        inline PID::pidParams_t getPitchAxisPidParameters(void) { return mPitchPid.getPidParameters(); }
        inline PID::pidParams_t getRollAxisPidParameters(void)  { return mRollPid.getPidParameters();  }
        inline PID::pidParams_t getYawAxisPidParameters(void)   { return mYawPid.getPidParameters();   }

        void setCommonPidParameters(float minOutputValue, float maxOutputValue, uint32_t pidUpdateTimeMs);
        /** @} */

        /// @returns The current flight angles computed by the sensors
        inline flightYPR_t getCurrentFlightAngles(void) const { return mCurrentAngles; }

        /**
         * Enables or Disables PID value logging
         * @param frequencyMs   The maximum frequency to log the messages (0 to turn it off)
         */
        void enablePidIoLogging(uint32_t frequencyMs);

    /* Next set of protected methods are meant to be called by the parent class */
    protected:
        /**
         * Runs filters on the sensor inputs.
         * The next step is to compute the PRY values using computePitchRollYawValues()
         */
        void runSensorInputFilters(void);

        /**
         * Computes the values of PRY from the sensor inputs using the AHRS algorithm
         * The next step is to run the PID and find out the throttle values using
         * computeThrottleValues()
         */
        void computePitchRollYawValues(void);

        /**
         * Computes the throttle values that should be applied on each motor
         * This runs the PID algorithm on current flight angles, and desired flight parameters
         * set by setFlightParameters() to yield the throttle values to be applied on each motor.
         *
         * The final step to fly is to apply the throttle values using applyThrottleValues()
         * You can obtain the computed values by calling the subclass's getMotorValues() method.
         */
        void computeThrottleValues(const uint32_t timeNowMs);

        /**
         * Applies the propeller values to the motors.
         * This calls the subclass method to retrieve its set values, and then calls
         * its virtual method to apply them.
         */
        inline void applyPropellerValues(void) { applyMotorValues(getMotorValues()); }

    protected:
        /// Protected constructor of this abstract class
        FlightController();

        /**
         * API to set flight parameters
         * @param [in] params   The flight parameters
         */
        inline void setFlightParameters(const flightParams_t& params)
        {
            mFlightControllerAngles = params;
        }

    private:
        flightParams_t mFlightControllerAngles;  ///< Input flight parameters
        flightYPR_t mCurrentAngles;              ///< The current flight angles

        PID mPitchPid;  ///< PID for pitch control
        PID mRollPid;   ///< PID for roll
        PID mYawPid;    ///< PID for yaw

        uint32_t mLogFrequencyMs;   ///< The log frequency in milliseconds

        // Allow private member access to register variables' telemetry
        ALLOW_FRIEND_TO_REGISTER_TLM();
};



#endif /* FLIGHT_CONTROLLER_HPP_ */
