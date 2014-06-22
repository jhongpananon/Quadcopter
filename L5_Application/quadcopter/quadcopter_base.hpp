/**
 * @file
 */
#ifndef QUADCOPTER_BASE_HPP_
#define QUADCOPTER_BASE_HPP_
#include <stdint.h>
#include "flight_controller.hpp"
#include "singleton_template.hpp"



/**
 * Quadcopter class
 *
 * This is the Quadcopter's "manager" which controls the flight controller class.
 * The flight is stabilized by the flight controller, and this class is responsible
 * for high level control of the flight.
 */
class QuadcopterBase : public FlightController
{
    /* Public types */
    public:
        /**
         * The GPS data type
         */
        typedef struct {
            float latitude;
            float longitude;
        } gpsData_t;

        /**
         * The quadcopter mode type
         */
        typedef enum {
            mode_invalid,
            mode_auto,
            mode_manual,
        } quadcopterMode_t;

    /* Public API */
    public:
        /**
         * Sets the flight parameters that are applied if the quadcopter is in the manual mode.
         * Certain conditions override the requested parameters, such as kill switch or low battery voltage.
         * @param [in] params   The flight parameters "requested" to be set.
         */
        void setFlightControl(const FlightController::flightParams_t& params);

        /**
         * Sets the current GPS coordinates of the Quadcopter
         * @param [in] data The GPS data
         */
        void setCurrentGpsCoordinates(const gpsData_t& data);

        /**
         * Set the destination coordinates to follow in autonomous mode
         * @param [in] data The GPS data
         */
        void setDestinationGpsCoordinates(const gpsData_t& data);

        /**
         * Sets the Quadcopter's mode
         * @param [in] mode     The quadcopter mode type
         */
        void setOperationMode(quadcopterMode_t mode);

        /**
         * @returns the mode set by setMode()
         */
        quadcopterMode_t getOperationMode(void) const;

        /**
         * Sets the battery charge percentage
         * @param [in] batteryPercent  The current battery charge in percentage
         */
        void setBatteryPercentage(uint8_t batteryPercent);

        /**
         * Sets the battery charge percentage that triggers low battery Quadcopter mode
         * @param [in] batteryPercent  The percent battery charge
         */
        void setLowBatteryTriggerPercentage(uint8_t batteryPercent);

        /**
         * Engages the "kill switch"
         */
        void engageKillSwitch(void);

        /**
         * Updates the status of the RC receiver to indicate if valid input is being
         * obtained or not.
         */
        void setRcReceiverStatus(bool isHealthy);

        /**
         * Flies the Quadcopter based on all the input data
         */
        void fly(void);

    protected:
        /// Protected constructor of this abstract class
        QuadcopterBase();

    private:
        /**
         * The Quadcopter's internal modes (imode) of operation
         */
        typedef enum {
            imode_invalid,                  ///< Invalid mode

            imode_low_battery,              ///< Low battery
            imode_no_rc_receiver,           ///< No RC receiver input
            imode_kill_switch_engaged,      ///< Kill swtich has been engaged

            imode_auto_mode_hover,          ///< Auto-mode, but hover only
            imode_auto_mode_follow_gps,     ///< Auto-mode, follow GPS

            imode_full_manual,              ///< Full manual mode
        } quadcopterInternalMode_t;

    private:

        quadcopterMode_t mQuadcopterMode;       ///< Quadcopter's mode
        quadcopterInternalMode_t mInternalMode; ///< Quadcopter's internal mode

        uint8_t mBatteryPercentage;             ///< Quadcopter's battery voltage
        uint8_t mLowBatteryTriggerPercentage;   ///< The trigger that enters low battery mode

        bool mRcReceiverIsHealthy;              ///< If valid input is being given by RC receiver remote
        bool mKillSwitchEngaged;                ///< Flag if kill switch has been engaged
        flightParams_t mRequestedFlightParams;  ///< Flight parameters being requested by the user (RC receiver)

        gpsData_t mCurrentGps;                  ///< Current GPS coordinates of the Quadcopter
        gpsData_t mDestinationGps;              ///< Destination GPS coordinates of the Quadcopter
};



#endif /* QUADCOPTER_BASE_HPP_ */
