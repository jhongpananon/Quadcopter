#include <stdint.h>

#include "scheduler_task.hpp"
#include "uart_dev.hpp"



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
        gps_task(UartDev *pGpsUart, const uint8_t priority);
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
        rc_remote_task(); ///< Disallow this constructor (no code is defined)
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
        battery_monitor_task(); ///< Disallow this constructor (no code is defined)
};
