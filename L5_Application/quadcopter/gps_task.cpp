#include "quad_tasks.hpp"

#include <stdio.h>
#include "FreeRTOS.h"
#include "semphr.h"

#include "file_logger.h"



/// Define the stack size this task is estimated to use
#define GPS_TASK_STACK_BYTES        (3 * 512)



gps_task::gps_task(const uint8_t priority, UartDev *pGpsUart) :
    scheduler_task("gps", GPS_TASK_STACK_BYTES, priority),
    mpGpsUart(pGpsUart)
{
    /* Use init() for memory allocation */
}

bool gps_task::init(void)
{
    bool success = true;

    if (success) {
        /* Init something here */
    }

    // Do not update task statistics (stack usage) too frequently
    setStatUpdateRate(5 * 60 * 1000);

    return success;
}

bool gps_task::run(void *p)
{
    const uint32_t maxGpsStringLen = 192;
    char buffer[maxGpsStringLen] = { 0 };

    // Assuming 1Hz GPS, we should receive the data within 1100ms
    uint32_t gpsTimeoutMs = 1100;

    /* Log an error if GPS data not retrieved within the expected time */
    if (!mpGpsUart->gets(&buffer[0], sizeof(buffer) - 1, OS_MS(gpsTimeoutMs))) {
        LOG_ERROR("GPS data not received within %u ms", gpsTimeoutMs);

        /* For test boards when GPS is not attached, we don't want to continously
         * poll for GPS data
         */
        gpsTimeoutMs = 60 * 1000;
    }
    else {
        /* Parse the GPS string */

        /* Set the GPS data on the Quadcopter class */
    }

    return true;
}
