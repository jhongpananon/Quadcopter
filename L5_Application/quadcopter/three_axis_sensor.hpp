/**
 * @file
 */
#ifndef SENSOR_DATA_HPP_
#define SENSOR_DATA_HPP_
#include <stdint.h>



/**
 * The class for sensor data.
 * The idea behind this class is that any kind of sensor such as acceleration and gyro
 * can be generalized.  Whether the sensor is 10-bit or 12-bit, we can compute its
 * real-world data (ie: gravity) more easily using this class.
 *
 * XXX : Future work should involve running simple filters on the input data
 * XXX : Future work may involve an acceleration sensor class inheriting this class
 *       to add more API, such as finding the gravity pull of each axis.
 */
class ThreeAxisSensor
{
    public:
        ThreeAxisSensor() : mMinimum(INT32_MIN), mMaximum(INT32_MAX), mXaxis(0), mYaxis(0), mZaxis(0)
        {
            /* Avoid heap allocation for this simple class */
        }

        /**
         * @{ Set the sensor data minimum and maximum values
         */
        void setMinimumValue(const int32_t value)   { mMinimum = value; }
        void setMaximumValue(const int32_t value)   { mMaximum = value; }
        /** @} */

        /**
         * @{ Get the sensor data values
         */
        int32_t getX(void) const { return mXaxis; }
        int32_t getY(void) const { return mYaxis; }
        int32_t getZ(void) const { return mZaxis; }
        void getAll(int32_t &x, int32_t &y, int32_t &z) const { x = mXaxis; y = mYaxis; z = mZaxis; }
        /** @} */

        /**
         * @{ Set the sensor data values
         */
        void setX(const int32_t value) { if (value >= mMinimum && value <= mMaximum) mXaxis = value; }
        void setY(const int32_t value) { if (value >= mMinimum && value <= mMaximum) mXaxis = value; }
        void setZ(const int32_t value) { if (value >= mMinimum && value <= mMaximum) mXaxis = value; }
        void setAll(const int32_t x, const int32_t y, const int32_t z)
        {
            setX(x);
            setY(y);
            setZ(z);
        }
        /** @} */

    private:
        int32_t mMinimum;   ///< Minimum value of the sensor
        int32_t mMaximum;   ///< Maximum value of the sensor
        int32_t mXaxis, mYaxis, mZaxis; ///< Actual sensor data values
};


#endif /* SENSOR_DATA_HPP_ */
