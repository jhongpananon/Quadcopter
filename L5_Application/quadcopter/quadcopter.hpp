#ifndef QUADCOPTER_HPP_
#define QUADCOPTER_HPP_
#include <stdint.h>



/**
 * Quadcopter class
 *
 * This is the Quadcopter's "manager"
 */
class Quadcopter
{
    /* Public types */
    public:
        typedef enum {
            mode_invalid,
            mode_auto,
            mode_manual,
        } quadcopter_mode_t;

    /* Public API */
    public:
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

    protected:
    private:
        quadcopter_mode_t mQuadcopterMode;  ///< Quadcopter's mode
        uint8_t mBatteryPercentage;         ///< Quadcopter's battery voltage
};


#endif /* QUADCOPTER_HPP_ */
