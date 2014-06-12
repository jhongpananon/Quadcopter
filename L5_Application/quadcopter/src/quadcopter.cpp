/**
 *
 */

#include <string.h>

#include "quadcopter.hpp"
#include "file_logger.h"



Quadcopter::Quadcopter() :
    mQuadcopterMode(mode_manual),
    mInternalMode(imode_full_manual),
    mBatteryPercentage(0),
    mLowBatteryTriggerPercentage(20),
    mRcReceiverIsHealthy(false),
    mKillSwitchEngaged(false)
{
    memset(&mCurrentGps, sizeof(mCurrentGps), 0);
    memset(&mDestinationGps, sizeof(mDestinationGps), 0);
    memset(&mRequestedFlightParams, sizeof(mRequestedFlightParams), 0);
}

void Quadcopter::setFlightParameters(const flightParams_t& params)
{
    mRequestedFlightParams = params;
}

void Quadcopter::setCurrentGpsCoordinates(const gpsData_t& data)
{
    mCurrentGps = data;
}

void Quadcopter::setDestinationGpsCoordinates(const gpsData_t& data)
{
    mDestinationGps = data;
}

void Quadcopter::setOperationMode(quadcopter_mode_t mode)
{
    mQuadcopterMode = mode;
}

Quadcopter::quadcopter_mode_t Quadcopter::getOperationMode(void) const
{
    return mQuadcopterMode;
}

void Quadcopter::setBatteryPercentage(uint8_t batteryPercent)
{
    const uint8_t max = 100;
    if (batteryPercent <= max) {
        mBatteryPercentage = batteryPercent;
    }
}

void Quadcopter::setLowBatteryTriggerPercentage(uint8_t batteryPercent)
{
    const uint8_t max = 100;
    if (batteryPercent <= max) {
        mLowBatteryTriggerPercentage = batteryPercent;
    }
}

void Quadcopter::engageKillSwitch(void)
{
    mKillSwitchEngaged = true;
}

void Quadcopter::setRcReceiverStatus(bool isHealthy)
{
    mRcReceiverIsHealthy = isHealthy;
}

void Quadcopter::fly(void)
{
    /* Ideas are documented here, need to collaborate and discuss:
     *
     *      - If kill switch engaged, go to "landing" mode, but wind down the propellers faster
     *          No way to get out of kill switch mode, must power off.
     *
     *      - If battery is low, go to "landing" mode
     *      - If RC receiver is not healthy, go to "landing" mode
     *
     *      - If autonomous mode, just hover at present location
     *      - If destination is set, slowly approach it
     *
     *      - Finally, if manual mode is engaged, apply that to the flight controller
     */

    /* The following transitions should take place regardless of current state:
     *
     *  - Kill switch is highest priority, and we switch to kill switch in all cases.
     *  - If kill switch not engaged, and battery is low, that's our next priority.
     *  - No RC receiver means we no longer have control of the Quadcopter, so what do we do?
     */
    if (mKillSwitchEngaged) {
        if (imode_kill_switch_engaged != mInternalMode) {
            mInternalMode = imode_kill_switch_engaged;

            LOG_INFO("Kill switch activated");
        }
    }
    else if (mBatteryPercentage < mLowBatteryTriggerPercentage) {
        if (imode_low_battery != mInternalMode) {
            mInternalMode = imode_low_battery;

            LOG_INFO("Low battery has been detected - %u/%u %%",
                     mLowBatteryTriggerPercentage, mBatteryPercentage);
        }
    }
    else if (!mRcReceiverIsHealthy) {

    }

    switch (mInternalMode)
    {
        case imode_low_battery:
        case imode_no_rc_receiver:
        case imode_kill_switch_engaged:

        case imode_auto_mode_hover:
        case imode_auto_mode_follow_gps:

        case imode_full_manual:
            mFlightController.setFlightParameters(mRequestedFlightParams);
            break;

        case imode_invalid:
        default:
            LOG_ERROR("Quadcopter is in invalid mode");
            break;
    }
}
