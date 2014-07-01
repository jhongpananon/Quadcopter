#include <stdio.h>

#include "FreeRTOS.h"
#include "semphr.h"

#include "quad_tasks.hpp"
#include "lpc_sys.h"
#include "shared_handles.h"

#include "io.hpp"
#include "file_logger.h"
#include "soft_timer.hpp"
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
    mQuadcopter(Quadcopter::getInstance()),
    mLowBatteryTriggerPercent(20),
    mLastCallMs(0),
    mNextLedUpdateTimeMs(0)
{
    /* Use init() for memory allocation */
}

bool quadcopter_task::init(void)
{
    bool success = true;
    FlightController &f = mQuadcopter;
    const uint32_t loopFrequencyMs = 1000 / QUADCOPTER_SENSOR_FREQUENCY;
    const uint32_t escFrequencyMs  = 1000 / QUADCOPTER_ESC_UPDATE_FREQUENCY;

    /* Set the PID's min and max PWM output along with the PID update rate */
    const float pwmMinPercent = 0;
    const float pwmMaxPercent = 100;
    f.setCommonPidParameters(pwmMinPercent, pwmMaxPercent, escFrequencyMs);

    /* TODO: Set min/max according to the particular sensor */
    f.mAccelerationSensor.setMinimumMaximumForAllAxis((4 * -1024), (4 * +1024));
            f.mGyroSensor.setMinimumMaximumForAllAxis((4 * -1024), (4 * +1024));
           f.mMagnoSensor.setMinimumMaximumForAllAxis((4 * -1024), (4 * +1024));

    // Do not update task statistics for this task since it may cause timing skew
    setStatUpdateRate(0);

    // Set the frequency of run() method
    setRunDuration(loopFrequencyMs);

    return success;
}

bool quadcopter_task::regTlm(void)
{
    bool success = true;

    /* Register all private variables of quadcopter class and its subclasses
     * Only quadcopterRegisterTelemetry() method has access to the private variables.
     */
    if (success) {
        success = quadcopterRegisterTelemetry();
    }

    /* Register the variable we want to preserve on the "disk" */
    if (success) {
        tlm_component *disk = tlm_component_get_by_name(DISK_TLM_NAME);
        success = TLM_REG_VAR(disk, mLowBatteryTriggerPercent, tlm_uint);
    }

    return success;
}

bool quadcopter_task::taskEntry(void)
{
    bool success = true;

    /* "Disk" data is restored at this point, so we set it to the Quadcopter class */
    mQuadcopter.setLowBatteryTriggerPercentage(mLowBatteryTriggerPercent);

    return success;
}

bool quadcopter_task::run(void *p)
{
    const uint32_t millis = sys_get_uptime_ms();

    /* Detect any "call rate" skew in case we are not getting called at precise timings */
    detectTimingSkew(millis);

    /* TODO Update the sensor values */

    /* Figure out what the Quadcopter should do */
    mQuadcopter.updateFlyLogic();

    /* Update the flight sensor system */
    mQuadcopter.updateSensorData(millis);

    /* Run the PID loop to apply propeller throttle values */
    mQuadcopter.updatePropellerValues(millis);

    /* Update status LEDs periodically */
    updateStatusLeds(millis);

    return true;
}

void quadcopter_task::detectTimingSkew(const uint32_t millis)
{
    /* We don't want to mark the timing skew during the first call (mLastCallMs will be zero at that time) */
    if (0 != mLastCallMs && (mLastCallMs + getRunDuration()) != millis)
    {
        const uint32_t maxLogMsgs = 10;
        if (mQuadcopter.getTimingSkewedCount() < maxLogMsgs)
        {
            LOG_ERROR("Quadcopter run() method timing is skewed");
            LOG_ERROR("Last call %u ms. This call: %u ms. Should have been %u ms.",
                      mLastCallMs, millis, getRunDuration());
        }
        mQuadcopter.incrTimingSkewedCount();
    }
    mLastCallMs = millis;
}

void quadcopter_task::updateStatusLeds(const uint32_t millis)
{
    /* Enumeration of LED number (1-4) */
    enum {
        led_error = 1,
        led_gps = 2,
    };

    /* Roughly update the LEDs at the rate givey by mLedUpdateRateMs */
    if (millis > mNextLedUpdateTimeMs)
    {
        mNextLedUpdateTimeMs = millis + mLedUpdateRateMs;

        LE.set(led_error, (mQuadcopter.getTimingSkewedCount() > 0) );
        LE.set(led_gps,   mQuadcopter.getGpsStatus());
    }
}
