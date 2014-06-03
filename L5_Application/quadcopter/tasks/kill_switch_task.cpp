/**
 * @file
 */
#include "quad_tasks.hpp"
#include "file_logger.h"
#include "wireless.h"



/// Define the stack size this task is estimated to use
#define KILL_SWITCH_TASK_STACK_BYTES        (3 * 512)



kill_switch_task::kill_switch_task(const uint8_t priority) :
    scheduler_task("killsw", KILL_SWITCH_TASK_STACK_BYTES, priority)
{
    /* Use init() for memory allocation */
}

bool kill_switch_task::init(void)
{
    bool success = true;

    if (success) {
        /* Init something here */
    }

    // Do not update task statistics (stack usage) too frequently
    setStatUpdateRate(5 * 60 * 1000);

    return success;
}

bool kill_switch_task::run(void *p)
{
    mesh_packet_t pkt;

    if (wireless_get_rx_pkt(&pkt, portMAX_DELAY))
    {
        /* TODO: If any wireless packet (over nordic) comes in just kill the quadcopter */

        LOG_WARN("Kill switch engaged!");
    }

    return true;
}
