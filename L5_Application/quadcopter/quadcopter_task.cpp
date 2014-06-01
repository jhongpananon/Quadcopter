#include "quad_tasks.hpp"

#include "FreeRTOS.h"
#include "semphr.h"



/// Define the stack size this task is estimated to use
#define QUADCOPTER_TASK_STACK_BYTES        (8 * 512)



quadcopter_task::quadcopter_task(const uint8_t priority) :
    scheduler_task("quadcopter", QUADCOPTER_TASK_STACK_BYTES, priority)
{
    /* Use init() for memory allocation */
}

bool quadcopter_task::init(void)
{
    bool success = true;

    if (success) {
        /* Init whatever you want here */
    }

    // Do not update task statistics (stack usage) too frequently
    setStatUpdateRate(1 * 60 * 1000);

    return success;
}

bool quadcopter_task::run(void *p)
{
    const uint32_t timeoutMs = 100;
    if (!xSemaphoreTake(getSharedObject(shared_SensorDataReadySemaphore), OS_MS(timeoutMs))) {
        /* Log an error */
    }

    /* Set the sensor values */

    /* Run the filters */

    /* Fly the quadcopter */

    return true;
}
