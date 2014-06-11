/**
 * @file
 */
#include <stdint.h>

#include "scheduler_task.hpp"

#include "sampler.hpp"
#include "uart_dev.hpp"
#include "three_axis_sensor.hpp"



/**
 * This is the sensor OS task.
 * The objective is to read all of the sensor values, and pass them
 * on to the flight controller for processing
 *
 * @ingroup Quadcopter Tasks
 */
class sensor_task : public scheduler_task
{
    public:
        sensor_task(const uint8_t priority);
        bool init(void);
        bool run(void *p);

    protected:
    private:
        sensor_task(); ///< Disallow this constructor (no code is defined)

        ThreeAxisSensor mAcceleration;  ///< Acceleration sensor data
        ThreeAxisSensor mGyro;          ///< Gyroscope sensor data
        ThreeAxisSensor mMagno;         ///< Magnetometer sensor data
};



/**
 * This is the Quadcopter OS task.
 * This processes the raw sensor values through various filters, and applies
 * the flight controller inputs to fly the quadcopter.
 *
 * @ingroup Quadcopter Tasks
 */
class quadcopter_task : public scheduler_task
{
    public:
        quadcopter_task(const uint8_t priority);
        bool init(void);
        bool run(void *p);

    protected:
    private:
        quadcopter_task(); ///< Disallow this constructor (no code is defined)
};



/**
 * GPS task.
 * The objective of this task is to read the GPS data, parse it, and set
 * it on the Quadcopter class.
 *
 * @ingroup Quadcopter Tasks
 */
class gps_task : public scheduler_task
{
    public:
        /**
         * Constructor of the task.
         * @param [in] pGpsUart     The UART pointer connected to the GPS
         * @param [in] priority     The priority of this task
         *
         * @note pGpsUart's UART should already be initialized at the GPS baudrate
         */
        gps_task(const uint8_t priority, UartDev *pGpsUart);
        bool init(void);
        bool run(void *p);

    protected:
    private:
        gps_task(); ///< Disallow this constructor (no code is defined)

        UartDev *mpGpsUart;     ///< The UART used for the GPS
};



/**
 * This is the RC remote receiver OS task.
 * The objective is to decoded/read the input values of the RC receiver
 * and set the values to the flight controller class.
 *
 * @ingroup Quadcopter Tasks
 */
class rc_remote_task : public scheduler_task
{
    public:
        rc_remote_task(const uint8_t priority);
        bool init(void);
        bool run(void *p);

    protected:
    private:
        int8_t getNormalizedValue(const uint32_t &pulseWidthUs);

        rc_remote_task(); ///< Disallow this constructor (no code is defined)

        int8_t mPitch;      ///< Last converted pitch input
        int8_t mRoll;       ///< Last converted roll input
        int8_t mYaw;        ///< Last converted yaw input
        uint8_t mThrottle;  ///< Last converted throttle input
        static const uint32_t mscMaxPulseWidthUs = 2 * 1000;
};



/**
 * This is the battery monitor task.
 * This monitors the battery voltage and sets the value to the quadcopter task.
 *
 * @ingroup Quadcopter Tasks
 */
class battery_monitor_task : public scheduler_task
{
    public:
        battery_monitor_task(const uint8_t priority);
        bool init(void);
        bool run(void *p);

    protected:
    private:
        battery_monitor_task();       ///< Disallow this constructor (no code is defined)
        float mLowestVoltage;         ///< Lowest battery voltage
        float mHighestVoltage;        ///< Highest battery voltage
        float mVoltageDeltaForLog;    ///< Data is logged if previous voltage delta is larger than this

        /**
         * We take multiple ADC samples before we take the average.
         * Note that we need uint32_t rather than the adequate uint16_t for 12-bit ADC
         * because when we sum and average using this class, it may overflow.
         */
        Sampler<uint32_t> mAdcSamples;

        /// The frequency at which we collect the samples in milliseconds
        static const int mSampleFrequencyMs = 250;

        /**
         * The number of samples to take before we average it and use it to compute the voltage
         * 250 * 12 = 3 seconds
         */
        static const int mNumAdcSamplesBeforeVoltageUpdate = 12;
};



/**
 * This is the "kill switch" task
 *
 * @ingroup Quadcopter Tasks
 */
class kill_switch_task : public scheduler_task
{
    public:
        kill_switch_task(const uint8_t priority);
        bool init(void);
        bool run(void *p);

    protected:
    private:
        kill_switch_task(); ///< Disallow this constructor (no code is defined)
};
