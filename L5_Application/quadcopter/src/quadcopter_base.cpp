/**
 *
 */

#include <string.h>

#include "quadcopter_base.hpp"
#include "file_logger.h"



QuadcopterBase::QuadcopterBase() :
    mQuadcopterMode(mode_manual),
    mInternalMode(imode_full_manual),
    mBatteryPercentage(100),            /* Assume 100% until changed otherwise */
    mLowBatteryTriggerPercentage(20),   /* Default to 20% until changed otherwise */
    mRcReceiverIsHealthy(true),         /* Assume receiver is healthy until changed */
    mKillSwitchEngaged(false)
{
    memset(&mCurrentGps, sizeof(mCurrentGps), 0);
    memset(&mDestinationGps, sizeof(mDestinationGps), 0);
    memset(&mRequestedFlightParams, sizeof(mRequestedFlightParams), 0);
}

void QuadcopterBase::setFlightControl(const FlightController::flightParams_t& params)
{
    mRequestedFlightParams = params;
}

void QuadcopterBase::setCurrentGpsCoordinates(const gpsData_t& data)
{
    mCurrentGps = data;
}

void QuadcopterBase::setDestinationGpsCoordinates(const gpsData_t& data)
{
    mDestinationGps = data;
}

void QuadcopterBase::setOperationMode(quadcopterMode_t mode)
{
    mQuadcopterMode = mode;
}

QuadcopterBase::quadcopterMode_t QuadcopterBase::getOperationMode(void) const
{
    return mQuadcopterMode;
}

void QuadcopterBase::setBatteryPercentage(uint8_t batteryPercent)
{
    const uint8_t max = 100;
    if (batteryPercent <= max) {
        mBatteryPercentage = batteryPercent;
    }
}

void QuadcopterBase::setLowBatteryTriggerPercentage(uint8_t batteryPercent)
{
    const uint8_t max = 100;
    if (batteryPercent <= max) {
        mLowBatteryTriggerPercentage = batteryPercent;
    }
}

void QuadcopterBase::engageKillSwitch(void)
{
    mKillSwitchEngaged = true;
}

void QuadcopterBase::setRcReceiverStatus(bool isHealthy)
{
    mRcReceiverIsHealthy = isHealthy;
}

void QuadcopterBase::fly(void)
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

            LOG_INFO_SIMPLE("Kill switch activated");
        }
    }
    else if (mBatteryPercentage < mLowBatteryTriggerPercentage) {
        if (imode_low_battery != mInternalMode) {
            mInternalMode = imode_low_battery;

            LOG_INFO_SIMPLE("Low battery has been detected - %u/%u %%",
                            mBatteryPercentage, mLowBatteryTriggerPercentage);
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
            FlightController::setFlightParameters(mRequestedFlightParams);
            break;

        case imode_invalid:
        default:
            LOG_ERROR("Quadcopter is in invalid mode");
            break;
    }
}
