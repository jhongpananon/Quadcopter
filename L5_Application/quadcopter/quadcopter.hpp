/**
 *
 */
#ifndef QUADCOPTER_HPP_
#define QUADCOPTER_HPP_
#include <stdint.h>
#include "flight_controller.hpp"
#include "singleton_template.hpp"



/**
 * Quadcopter class
 *
 * This is the Quadcopter's "manager"
 */
class Quadcopter : public SingletonTemplate<Quadcopter>
{
    /* Public types */
    public:
        typedef enum {
            mode_invalid,
            mode_auto,
            mode_manual,
        } quadcopter_mode_t;

    /* Public members */
        FlightController mFlightController;

    /* Public API */
    public:
        void setFlightParameters(const flight_params_t& params)
        {
            mRequestedFlightParams = params;
        }

        /**
         * Sets the GPS coordinates
         * @param [in] longitude
         * @param [in] latitude
         */
        void setGps(float longitude, float latitude)
        {

        }

        /**
         * Sets the Quadcopter's mode
         * @param [in] mode     The quadcopter mode type
         */
        void setMode(quadcopter_mode_t mode);

        /**
         * @returns the mode set by setMode()
         */
        quadcopter_mode_t getMode(void);

        /**
         * Sets the battery charge percentage
         */
        void setBatteryPercentage(uint8_t batteryPercent)
        {
            const uint8_t max = 100;
            if (batteryPercent <= max) {
                mBatteryPercentage = batteryPercent;
            }
        }

        void engageKillSwitch(void)
        {

        }

        void updateRcReceiverStatus(bool isHealthy)
        {
            mRcReceiverIsHealthy = isHealthy;
        }

    protected:
    private:
        /// Private constructor for singleton class
        Quadcopter() : mQuadcopterMode(mode_manual), mBatteryPercentage(0), mRcReceiverIsHealthy(false)
        {

        }

        ///< Friend class used for Singleton Template
        friend class SingletonTemplate<Quadcopter>;

        quadcopter_mode_t mQuadcopterMode;      ///< Quadcopter's mode
        uint8_t mBatteryPercentage;             ///< Quadcopter's battery voltage
        bool mRcReceiverIsHealthy;              ///< If valid input is being given by RC receiver remote
        flight_params_t mRequestedFlightParams; ///< Flight parameters being requested
};


#endif /* QUADCOPTER_HPP_ */
