#include <stdio.h>

#include "FreeRTOS.h"
#include "semphr.h"

#include "quad_tasks.hpp"
#include "lpc_sys.h"
#include "shared_handles.h"

#include "io.hpp"
#include "file_logger.h"
#include "c_tlm_var.h"



/// Define the stack size this task is estimated to use
#define QUADCOPTER_TASK_STACK_BYTES        (3 * 512)

/// Define the frequency of updating sensors and running the AHRS sensor loop
#define QUADCOPTER_SENSOR_FREQUENCY       (250)

/// Define the frequency at which the ESC (electronic speed controllers) will update
#define QUADCOPTER_ESC_UPDATE_FREQUENCY   (125)



// This is the only method that has access to private members, which is used to register telemetry
bool quadcopterRegisterTelemetry(void)
{
    bool success = true;
    const char * tlmName = "quadcopter_vars";
    Quadcopter &q = Quadcopter::getInstance();
    tlm_component *quad = tlm_component_get_by_name(tlmName);

    /* If telemetry component not found, then create it */
    if (NULL == quad) {
        quad = tlm_component_add(tlmName);
    }
    if (success) {
        success = (NULL != quad);
    }

    // Quick hack to register variables
    #define TLM_REG_QUAD_VAR(name, var, type) \
                tlm_variable_register(quad, name, &(var), sizeof(var), 1, type);

    /* Register Pitch, Roll, and Yaw axis PIDs.
     * These shouldn't be directly modified by the user through telemetry because the PID
     * class manipulates these based on the timings.  terminal command should be used instead
     * to safely modify these parameters.
     */
    if (success) success = TLM_REG_QUAD_VAR("pid_pitch_kp", q.mPitchPid.mPidParams.kp, tlm_float);
    if (success) success = TLM_REG_QUAD_VAR("pid_pitch_ki", q.mPitchPid.mPidParams.ki, tlm_float);
    if (success) success = TLM_REG_QUAD_VAR("pid_pitch_kd", q.mPitchPid.mPidParams.kd, tlm_float);

    if (success) success = TLM_REG_QUAD_VAR("pid_roll_kp", q.mRollPid.mPidParams.kp, tlm_float);
    if (success) success = TLM_REG_QUAD_VAR("pid_roll_ki", q.mRollPid.mPidParams.ki, tlm_float);
    if (success) success = TLM_REG_QUAD_VAR("pid_roll_kd", q.mRollPid.mPidParams.kd, tlm_float);

    if (success) success = TLM_REG_QUAD_VAR("pid_yaw_kp", q.mYawPid.mPidParams.kp, tlm_float);
    if (success) success = TLM_REG_QUAD_VAR("pid_yaw_ki", q.mYawPid.mPidParams.ki, tlm_float);
    if (success) success = TLM_REG_QUAD_VAR("pid_yaw_kd", q.mYawPid.mPidParams.kd, tlm_float);

    /* Register the current angles being computed */
    if (success) success = TLM_REG_QUAD_VAR("current_pitch", q.mCurrentAngles.pitch, tlm_int);
    if (success) success = TLM_REG_QUAD_VAR("current_roll",  q.mCurrentAngles.roll, tlm_int);
    if (success) success = TLM_REG_QUAD_VAR("current_yaw",   q.mCurrentAngles.yaw, tlm_int);

    /* Register the requested angles by the user */
    if (success) success = TLM_REG_QUAD_VAR("requested_pitch", q.mRequestedFlightParams.angle.pitch, tlm_int);
    if (success) success = TLM_REG_QUAD_VAR("requested_roll",  q.mRequestedFlightParams.angle.roll, tlm_int);
    if (success) success = TLM_REG_QUAD_VAR("requestedt_yaw",  q.mRequestedFlightParams.angle.yaw, tlm_int);

    /* Register the latest angles applied by the flight controller */
    if (success) success = TLM_REG_QUAD_VAR("applied_pitch", q.mFlightControllerAngles.angle.pitch, tlm_int);
    if (success) success = TLM_REG_QUAD_VAR("applied_roll",  q.mFlightControllerAngles.angle.roll, tlm_int);
    if (success) success = TLM_REG_QUAD_VAR("applied_yaw",   q.mFlightControllerAngles.angle.yaw, tlm_int);

    return success;
}

quadcopter_task::quadcopter_task(const uint8_t priority) :
    scheduler_task("quadcopter", QUADCOPTER_TASK_STACK_BYTES, priority),
    mLowBatteryTriggerPercent(20),
    mLastCallMs(0)
{
    /* Use init() for memory allocation */
}

bool quadcopter_task::init(void)
{
    bool success = true;
    FlightController &f = Quadcopter::getInstance();
    const uint32_t loopFrequencyMs = 1000 / QUADCOPTER_SENSOR_FREQUENCY;
    const uint32_t escFrequencyMs  = 1000 / QUADCOPTER_ESC_UPDATE_FREQUENCY;

    /* Register the variable we want to preserve on the "disk" */
    if (success) {
        tlm_component *disk = tlm_component_get_by_name(DISK_TLM_NAME);
        if (success) success = TLM_REG_VAR(disk, mLowBatteryTriggerPercent, tlm_uint);
    }

    /* TODO: Set min/max according to the particular sensor */
    if (success) {
        const float pwmMinPercent = 0;
        const float pwmMaxPercent = 100;
        f.setCommonPidParameters(pwmMinPercent, pwmMaxPercent, escFrequencyMs);

        f.mAccelerationSensor.setMinimumMaximumForAllAxis((4 * -1024), (4 * +1024));
                f.mGyroSensor.setMinimumMaximumForAllAxis((4 * -1024), (4 * +1024));
               f.mMagnoSensor.setMinimumMaximumForAllAxis((4 * -1024), (4 * +1024));
    }

    // Do not update task statistics (stack usage) too frequently
    setStatUpdateRate(1 * 60 * 1000);

    // Set the frequency of run() method
    setRunDuration(loopFrequencyMs);

    return success;
}

bool quadcopter_task::regTlm(void)
{
    return quadcopterRegisterTelemetry();
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
    Quadcopter &q = Quadcopter::getInstance();
    const uint32_t millis = sys_get_uptime_ms();

    /* Detect any "call rate" skew in case we are not getting called at precise timings */
    if (0 != mLastCallMs && (mLastCallMs + getRunDuration()) != millis)
    {
        if (!q.getTimingSkewedFlag())
        {
            LOG_ERROR("Quadcopter run() method was not called at precise timings");
            LOG_ERROR("Expected %u ms, actual %u ms", (mLastCallMs + getRunDuration()), millis);
        }
        q.setTimingSkewedFlag(true);
    }
    mLastCallMs = millis;

    /* TODO Update the sensor values */

    /* Figure out what the Quadcopter should do */
    q.updateFlyLogic();

    /* Update the flight sensor system */
    q.updateSensorData(millis);

    /* Run the PID loop to apply propeller throttle values */
    q.updatePropellerValues(millis);

    /* Update any status LEDs */
    enum {
        led_error = 1,
        led_gps = 2,
    };

    LE.set(led_error, q.getTimingSkewedFlag());
    LE.set(led_gps,   q.getGpsStatus());

    return true;
}
