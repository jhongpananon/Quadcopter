/**
 * @file
 */
#ifndef SENSOR_DATA_HPP_
#define SENSOR_DATA_HPP_
#include <stdint.h>



/**
 * A single axis sensor
 */
template <typename TYPE>
class OneAxisSensor
{
    public:
        OneAxisSensor() : mRawValue(0), mOffset(0), mMinimum(0), mMaximum(0)
        {

        }

        /**
         * @returns the value of the sensor after factoring the offset
         */
        TYPE get(void) const
        {
            TYPE val = mRawValue - mOffset;

            if (val < mMinimum) {
                val = mMinimum;
            }
            else if (val > mMaximum) {
                val = mMaximum;
            }

            return val;
        }

        /**
         * Sets the raw value of the sensor
         */
        void set(const TYPE& value)
        {
            mRawValue = value;
        }

        /**
         * @{ Set the sensor data minimum and maximum values
         */
        void setMinimumValue(const TYPE& value)   { mMinimum = value; }
        void setMaximumValue(const TYPE& value)   { mMaximum = value; }
        /** @} */

        /**
         * Sets the offset value of the sensor
         */
        void setOffset(const TYPE& value) { mOffset = value; }

    private:
        TYPE mRawValue;   ///< Actual (raw) value
        TYPE mOffset;     ///< Offset value
        TYPE mMinimum;    ///< The lowest value that can be set
        TYPE mMaximum;    ///< The highest value that can be set
};

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
    /* Public variables */
    public:
        OneAxisSensor<int32_t> x;
        OneAxisSensor<int32_t> y;
        OneAxisSensor<int32_t> z;

    public:
        ThreeAxisSensor()
        {
            /* Avoid heap allocation for this simple class */
        }

        /// Get the sensor data values
        void getAll(int32_t &xval, int32_t &yval, int32_t &zval) const
        {
            xval = x.get();
            yval = y.get();
            zval = z.get();
        }

        /// Sets the sensor data values
        void setAll(const int32_t &xval, const int32_t &yval, const int32_t &zval)
        {
            x.set(xval);
            y.set(yval);
            z.set(zval);
        }

        /// Sets the minimum and maximum values for all axis
        void setMinimumMaximumForAllAxis(const int32_t &min, const int32_t &max)
        {
            x.setMinimumValue(min);
            x.setMaximumValue(max);

            y.setMinimumValue(min);
            y.setMaximumValue(max);

            z.setMinimumValue(min);
            z.setMaximumValue(max);
        }

    private:

};


#endif /* SENSOR_DATA_HPP_ */
