/**
 * @file
 */

#include "command_handler.hpp"
#include "quadcopter.hpp"
#include "file_logger.h"



CMD_HANDLER_FUNC(quadcopterPidChangeHandler)
{
    bool handled = true;
    const char * pAxisName = NULL;
    PID::pidParams_t params;
    Quadcopter &q = Quadcopter::getInstance();

    if (cmdParams.beginsWithIgnoreCase("get"))
    {
        PID::pidParams_t p = q.getPitchAxisPidParameters();
        PID::pidParams_t r = q.getRollAxisPidParameters();
        PID::pidParams_t y = q.getYawAxisPidParameters();

        output.printf("Pitch Axis: %f(kp) %f(ki) %f(kd)\n", p.kp, p.ki, p.kd);
        output.printf(" Roll Axis: %f(kp) %f(ki) %f(kd)\n", r.kp, r.ki, r.kd);
        output.printf("  Yaw Axis: %f(kp) %f(ki) %f(kd)\n", y.kp, y.ki, y.kd);
    }
    /* Must be able to parse 3 floating points */
    else if (3 != cmdParams.scanf("%*s %f %f %f", &params.kp, &params.ki, &params.kd))
    {
        output.printf("ERROR: Need 3 parameters for <kp> <ki> <kd>\n");
    }
    else if (cmdParams.beginsWithIgnoreCase("pitch"))
    {
        pAxisName = "pitch";
        q.setPitchAxisPidParameters(params);
    }
    else if (cmdParams.beginsWithIgnoreCase("roll"))
    {
        pAxisName = "roll";
        q.setRollAxisPidParameters(params);
    }
    else if (cmdParams.beginsWithIgnoreCase("yaw"))
    {
        pAxisName = "yaw";
        q.setYawAxisPidParameters(params);
    }
    else
    {
        handled = false;
    }

    /* Output the response if we set PID parameters correctly */
    if (NULL != pAxisName)
    {
        output.printf("Set %s PID parameters to: %f(kp) %f(ki) %f(kd)\n", pAxisName, params.kp, params.ki, params.kd);
    }

    return handled;
}

CMD_HANDLER_FUNC(quadcopterPidLogHandler)
{
    bool handled = true;

    if (cmdParams.beginsWithIgnoreCase("status"))
    {
        output.printf("Blocked logger calls: %i\n", logger_get_blocked_call_count());
    }
    else if (cmdParams.beginsWithIgnoreCase("pid"))
    {
        const uint32_t minMs = 20;
        uint32_t ms = 0;
        cmdParams.scanf("%*s %u", &ms);
        if (ms > 0 && ms < minMs) {
            ms = minMs;
        }

        Quadcopter::getInstance().enablePidIoLogging(ms);
        output.printf("%s PID logging every %u ms\n", (ms > 0) ? "Enabled" : "Disabled", ms);
    }
    else
    {
        handled = false;
    }

    return handled;
}
