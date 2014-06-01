#ifndef QUADCOPTER_HPP_
#define QUADCOPTER_HPP_
#include <stdint.h>



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
        void setGps(float longitude, float latitude);
        void setMode(quadcopter_mode_t mode);
        quadcopter_mode_t getMode(void);

    protected:
    private:
        quadcopter_mode_t mQuadcopterMode;
};


#endif /* QUADCOPTER_HPP_ */
