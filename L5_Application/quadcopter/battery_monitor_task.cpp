#include "quad_tasks.hpp"



/// Define the stack size this task is estimated to use
#define BATTERY_TASK_STACK_BYTES        (3 * 512)



battery_monitor_task::battery_monitor_task(const uint8_t priority) :
    scheduler_task("battery", BATTERY_TASK_STACK_BYTES, priority)
{
    /* Use init() for memory allocation */
}

bool battery_monitor_task::init(void)
{
    bool success = true;

    if (success) {
        /* Init the ADC pin here */
    }

    // Do not update task statistics (stack usage) too frequently
    setStatUpdateRate(5 * 60 * 1000);

    // Monitor the battery at a very slow rate
    setRunDuration(1000);

    return success;
}

bool battery_monitor_task::run(void *p)
{
    /* Read the ADC here */

    /* Convert ADC to voltage value */

    /* Convert voltage value to percentage value */

    /* Set the value to the quadcopter class */


    return true;
}
