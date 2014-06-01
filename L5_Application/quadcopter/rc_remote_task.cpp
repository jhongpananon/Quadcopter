#include "quad_tasks.hpp"

#include "FreeRTOS.h"
#include "semphr.h"

#include "shared_handles.h"
#include "file_logger.h"



/// Define the stack size this task is estimated to use
#define RC_RX_TASK_STACK_BYTES        (3 * 512)



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
        addSharedObject(shared_RcReceiverSemaphore, xSemaphoreCreateBinary());
    }

    // Do not update task statistics (stack usage) too frequently
    setStatUpdateRate(5 * 60 * 1000);

    return success;
}

bool rc_remote_task::run(void *p)
{
    const TickType_t timeout = OS_MS(1000);

    /* Wait for semaphore (upon pin change interrupt) */
    if (!xSemaphoreTake(getSharedObject(shared_RcReceiverSemaphore), timeout)) {
        LOG_ERROR("RC receiver failed!");
        return false;
    }

    /* Capture the time */

    /* Decode all the inputs */

    /* Set the inputs to the flight controller class */

    return true;
}
