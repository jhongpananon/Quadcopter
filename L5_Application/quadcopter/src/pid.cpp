/**
 * @file
 */
#include "pid.hpp"



PID::PID() :
        kp(0), ki(0), kd(0),
        mPidOutput(0),
        mPidSetpoint(0),
        mLastTimeMs(0),
        mSampleTimeMs(1000),
        mIntegralTerm(0),
        mLastInput(0),
        mPidProcessingIsOn(false),
        mPidControllerDirection(pid_direction_positive),
        mOutputMin(0),
        mOutputMax(0)
{

}

float PID::compute(const float setpointValue, const float presentInputValue, const uint32_t timeNowMs)
{
    /* If PID processing is not ON, we shouldn't update the PID state */
    if (!mPidProcessingIsOn)
    {
        return mPidOutput;
    }

    /* Only process the PID loop if enough time has elapsed since last computation.
     * The exception is when the user forces the PID to process its loop.
     */
    if ((timeNowMs - mLastTimeMs) >= mSampleTimeMs)
    {
        mPidSetpoint = setpointValue;

        /* Compute all the working error variables */
        const float error = mPidSetpoint - presentInputValue;
        mIntegralTerm += (ki * error);

        /* Cap the integral term to avoid PID from using unusable values.
         * http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-reset-windup/
         */
        if (mIntegralTerm > mOutputMax) {
            mIntegralTerm = mOutputMax;
        }
        else if (mIntegralTerm < mOutputMin) {
            mIntegralTerm = mOutputMin;
        }

        /* Avoid the derivative kick:
         * http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
         */
        const float dInput = (presentInputValue - mLastInput);

        /* Compute PID Output */
        mPidOutput = (kp * error) + mIntegralTerm - (kd * dInput);

        /* Cap the PID from using unusable values */
        if (mPidOutput > mOutputMax) {
            mPidOutput = mOutputMax;
        }
        else if (mPidOutput < mOutputMin) {
            mPidOutput = mOutputMin;
        }

        /* Remember some variables for next time */
        mLastInput = presentInputValue;
        mLastTimeMs = timeNowMs;
    }

    return mPidOutput;
}

void PID::setPidParameters(float Kp, float Ki, float Kd)
{
    if (Kp < 0 || Ki < 0 || Kd < 0) {
        return;
    }

    const float SampleTimeInSec = ((float) mSampleTimeMs) / 1000;
    kp = Kp;
    ki = Ki * SampleTimeInSec;
    kd = Kd / SampleTimeInSec;

    /* Negative the PID parameters if the direction is negative
     * http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-direction/
     */
    if (pid_direction_negative == mPidControllerDirection)
    {
        kp = (0 - kp);
        ki = (0 - ki);
        kd = (0 - kd);
    }
}

void PID::setSampleTime(const uint32_t newSampleTimeMs)
{
    if (newSampleTimeMs > 0)
    {
        float ratio = (float) newSampleTimeMs / (float) mSampleTimeMs;
        ki *= ratio;
        kd /= ratio;
        mSampleTimeMs = (int) newSampleTimeMs;
    }
}

void PID::setOutputLimits(float min, float max)
{
    if (min > max) {
        return;
    }

    mOutputMin = min;
    mOutputMax = max;

    if (mPidOutput > mOutputMax) {
        mPidOutput = mOutputMax;
    }
    else if (mPidOutput < mOutputMin) {
        mPidOutput = mOutputMin;
    }

    if (mIntegralTerm > mOutputMax) {
        mIntegralTerm = mOutputMax;
    }
    else if (mIntegralTerm < mOutputMin) {
        mIntegralTerm = mOutputMin;
    }
}

void PID::setMode(pidMode_t mode, float latestInput)
{
    bool newAuto = (mode == pid_automatic);

    /* If we just went from pid_manual to auto */
    if (newAuto == !mPidProcessingIsOn)
    {
        init(latestInput);
    }

    mPidProcessingIsOn = newAuto;
}

void PID::init(float latestInput)
{
    mLastInput = latestInput;
    mIntegralTerm = mPidOutput;

    if (mIntegralTerm > mOutputMax) {
        mIntegralTerm = mOutputMax;
    }
    else if (mIntegralTerm < mOutputMin) {
        mIntegralTerm = mOutputMin;
    }
}

void PID::setPidDirection(pidDirection_t pidDirection)
{
    mPidControllerDirection = pidDirection;
}
