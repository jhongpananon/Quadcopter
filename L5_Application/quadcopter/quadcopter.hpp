/**
 * @file
 */

#ifndef QUADCOPTER_HPP_
#define QUADCOPTER_HPP_

#include "quadcopter_base.hpp"
#include "lpc_pwm.hpp"



/**
 * The quadcopter class
 */
class Quadcopter : public QuadcopterBase, public SingletonTemplate<Quadcopter>
{
    public:
        /**
         * @{ Pure virtual method overrides of the MotorControllerIface
         */
        void setNorthMotor(uint8_t throttleValuePercent);
        void setSouthMotor(uint8_t throttleValuePercent);
        void setEastMotor (uint8_t throttleValuePercent);
        void setWestMotor (uint8_t throttleValuePercent);
        /** @} */

    private:
        /// Private constructor for singleton class
        Quadcopter();

        PWM mNorthMotor;    ///< North motor PWM
        PWM mSouthMotor;    ///< South motor PWM
        PWM mEastMotor;     ///< East motor PWM
        PWM mWestMotor;     ///< West motor PWM

        ///< Friend class used for Singleton Template
        friend class SingletonTemplate<Quadcopter>;
};



#endif /* QUADCOPTER_HPP_ */
