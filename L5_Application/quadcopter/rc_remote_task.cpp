#include "quad_tasks.hpp"

#include "FreeRTOS.h"
#include "semphr.h"



/// Define the stack size this task is estimated to use
#define RC_RX_TASK_STACK_BYTES        (2 * 512)



rc_remote_task::rc_remote_task(const uint8_t priority) :
    scheduler_task("rcrx", RC_RX_TASK_STACK_BYTES, priority)
{
    /* Use init() for memory allocation */
}

bool rc_remote_task::init(void)
{
    bool success = true;

    if (success) {
        /* Init RC remote receiver input pin interrupts */
    }

    // Do not update task statistics (stack usage) too frequently
    setStatUpdateRate(5 * 60 * 1000);

    return success;
}

bool rc_remote_task::run(void *p)
{
    /* Wait for semaphore (upon pin change interrupt) */

    /* Capture the time */

    /* Decode all the inputs */

    /* Set the inputs to the flight controller class */

    vTaskDelay(1); /* Remove this after semaphores are setup */

    return true;
}
