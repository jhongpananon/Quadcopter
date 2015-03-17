/**
 *
 */

#include "quadcopter.hpp"



/// The frequency of the ESC (electronic speed controller)
#define ESC_FREQUENCY_HZ    500



Quadcopter::Quadcopter() :
    mNorthMotor(PWM::pwm1, ESC_FREQUENCY_HZ),
    mSouthMotor(PWM::pwm2, ESC_FREQUENCY_HZ),
    mEastMotor (PWM::pwm3, ESC_FREQUENCY_HZ),
    mWestMotor (PWM::pwm4, ESC_FREQUENCY_HZ)
{
    /* Nothing to do */
}

// Virtual method implementation
void Quadcopter::applyMotorValues(const motorValues_t& values)
{
    mNorthMotor.set(values.north);
    mSouthMotor.set(values.south);
    mEastMotor.set(values.east);
    mWestMotor.set(values.west);
}
