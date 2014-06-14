#include "quad_tasks.hpp"

#include "FreeRTOS.h"
#include "semphr.h"

#include "shared_handles.h"
#include "file_logger.h"
#include "c_tlm_var.h"



/// Define the stack size this task is estimated to use
#define QUADCOPTER_TASK_STACK_BYTES        (3 * 512)



quadcopter_task::quadcopter_task(const uint8_t priority) :
    scheduler_task("quadcopter", QUADCOPTER_TASK_STACK_BYTES, priority),
    mLowBatteryTriggerPercent(20)
{
    /* Use init() for memory allocation */
}

bool quadcopter_task::init(void)
{
    bool success = true;

    /* Register the variable we want to preserve on the "disk" */
    if (success) {
        tlm_component *disk = tlm_component_get_by_name(DISK_TLM_NAME);
        if (success) success = TLM_REG_VAR(disk, mLowBatteryTriggerPercent, tlm_uint);
    }

    // Do not update task statistics (stack usage) too frequently
    setStatUpdateRate(1 * 60 * 1000);

    return success;
}

bool quadcopter_task::taskEntry(void)
{
    bool success = true;

    /* "Disk" data is restored at this point, so we set it to the Quadcopter class */
    Quadcopter::getInstance().setLowBatteryTriggerPercentage(mLowBatteryTriggerPercent);

    return success;
}

bool quadcopter_task::run(void *p)
{
    const uint32_t timeoutMs = 100;
    Quadcopter &q = Quadcopter::getInstance();

    if (!xSemaphoreTake(getSharedObject(shared_SensorDataReadySemaphore), OS_MS(timeoutMs))) {
        LOG_ERROR("Unable to get sensor data input within %u ms", timeoutMs);
        return true;
    }

    /* Run the filters on the raw input received by the flight controller */
    q.mFlightController.runSensorInputFilters();

    /* Fly the quadcopter */
    q.fly();

    return true;
}
