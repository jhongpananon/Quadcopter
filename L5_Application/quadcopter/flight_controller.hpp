#ifndef FLIGHT_CONTROLLER_HPP_
#define FLIGHT_CONTROLLER_HPP_
#include <stdint.h>



/**
 * This is the flight controller class.
 * This allows the user to set the raw sensor values, process them, and apply
 * the user input to be able to fly the quadcopter.
 */
class FlightController
{
    public:
        void setAcceleration(uint32_t x, uint32_t y, uint32_t z);
        void setGyro(uint32_t x, uint32_t y, uint32_t z);
        void setMagno(uint32_t x, uint32_t y, uint32_t z);

        void runSmoothingFilter(void);
        void runAnotherFilter(void);
        void runPID(void);
        void runKalman(void);

        void setFlightParameters(uint32_t pitch, uint32_t roll, uint32_t yaw);
        void setLift(uint32_t percentage);

    protected:
    private:
};



#endif /* FLIGHT_CONTROLLER_HPP_ */
