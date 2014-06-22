/**
 *
 */

#include "quadcopter.hpp"



/// The frequency of the ESC (electronic speed controller)
#define ESC_FREQUENCY_HZ    100

Quadcopter::Quadcopter() :
    mNorthMotor(PWM::pwm1, ESC_FREQUENCY_HZ),
    mSouthMotor(PWM::pwm2, ESC_FREQUENCY_HZ),
    mEastMotor (PWM::pwm3, ESC_FREQUENCY_HZ),
    mWestMotor (PWM::pwm4, ESC_FREQUENCY_HZ)
{
    /* Nothing to do */
}

void Quadcopter::setNorthMotor(uint8_t throttleValuePercent) {   mNorthMotor.set(throttleValuePercent);  }
void Quadcopter::setSouthMotor(uint8_t throttleValuePercent) {   mSouthMotor.set(throttleValuePercent);  }
void Quadcopter::setEastMotor (uint8_t throttleValuePercent) {    mEastMotor.set(throttleValuePercent);  }
void Quadcopter::setWestMotor (uint8_t throttleValuePercent) {    mWestMotor.set(throttleValuePercent);  }
