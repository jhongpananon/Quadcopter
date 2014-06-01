#include "quad_tasks.hpp"

#include "FreeRTOS.h"
#include "semphr.h"

#include "shared_handles.h"



/// Define the stack size this task is estimated to use
#define SENSOR_TASK_STACK_BYTES     (3 * 512)



sensor_task::sensor_task(const uint8_t priority) :
    scheduler_task("sensor", SENSOR_TASK_STACK_BYTES, priority)
{
    /* Use init() for memory allocation */
}

bool sensor_task::init(void)
{
    bool success = true;
    SemaphoreHandle_t sensorDataReadySem = xSemaphoreCreateBinary();

    if (success) {
        success = (NULL != sensorDataReadySem);
    }

    if (success) {
        success = addSharedObject(shared_SensorDataReadySemaphore, sensorDataReadySem);
    }

    // Do not update task statistics (stack usage) too frequently
    setStatUpdateRate(5 * 60 * 1000);

    // 100Hz sensor loop
    setRunDuration(10);

    return success;
}

bool sensor_task::run(void *p)
{
    return true;
}
