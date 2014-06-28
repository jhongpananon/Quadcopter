/**
 *
 */

#include <string.h>

#include "flight_controller.hpp"



FlightController::FlightController()
{
    memset(&mFlightControllerAngles, 0, sizeof(mFlightControllerAngles));
    memset(&mCurrentAngles, 0, sizeof(mCurrentAngles));
}

void FlightController::setCommonPidParameters(float minOutputValue, float maxOutputValue, uint32_t pidUpdateTimeMs)
{
    mPitchPid.setOutputLimits(minOutputValue, maxOutputValue);
    mRollPid.setOutputLimits(minOutputValue, maxOutputValue);
    mYawPid.setOutputLimits(minOutputValue, maxOutputValue);

    mPitchPid.setSampleTime(pidUpdateTimeMs);
    mRollPid.setSampleTime(pidUpdateTimeMs);
    mYawPid.setSampleTime(pidUpdateTimeMs);
}

void FlightController::runSensorInputFilters(void)
{

}

void FlightController::computePitchRollYawValues(void)
{

}

void FlightController::computeThrottleValues(const uint32_t timeNowMs)
{
    MotorControllerIface::motorValues_t values;
    const float throttle = (float) mFlightControllerAngles.throttle;

    /* Get the PRY values we need through the PID */
    const float pitchThrottle = mPitchPid.compute(mFlightControllerAngles.angle.pitch, mCurrentAngles.pitch, timeNowMs);
    const float rollThrottle = mRollPid.compute(mFlightControllerAngles.angle.roll, mCurrentAngles.roll, timeNowMs);
    const float yawThrottle = mYawPid.compute(mFlightControllerAngles.angle.yaw, mCurrentAngles.yaw, timeNowMs);

    /* Set the motor values that control the pitch
     * For example, if the desired pitch angle is 15deg (nose up), and actual is zero, then the
     * pitchThrottle value will be positive, and so we need to increase north motor and decrease
     * the south motor.
     */
    values.north = throttle + pitchThrottle;
    values.south = throttle - pitchThrottle;

    /* Set the motor values that control the roll
     * For example, if desired pitch angle is 15deg (to the right), and actual is zero, then the
     * rollThrottle value will be positive, and so we need to increase the west motor and decrease
     * the east motor.
     */
    values.east = throttle - rollThrottle;
    values.west = throttle + rollThrottle;

    /* TODO: Alter the motor values to control the yaw
     * We can do this later once stable flight has been achieved
     */
    (void) yawThrottle; // Avoid the unused variable warning for now

    saveMotorValues(values);
}
