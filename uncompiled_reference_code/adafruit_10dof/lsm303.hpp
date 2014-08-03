/*
 * lsm303.hpp
 *
 *  Created on: Jul 20, 2014
 *      Author: pardeep
 */

#ifndef LSM303_HPP_
#define LSM303_HPP_


#include "i2c2_device.hpp"
#include <singleton_template.hpp>


#define LSM303_ID                     (0b11010100)


class lsm303Accelero : private i2c2_device, public SingletonTemplate<lsm303Accelero>
{
    private:
        #define LSM303_ADDRESS_ACCEL         (0x32)
        #define LSM303_GRAVITY_EARTH         (9.80665F)              /**< Earth's gravity in m/s^2 */
        #define LSM303_SENSORS_GRAVITY_MOON  (1.6F)                  /**< The moon's gravity in m/s^2 */
        #define LSM303_GRAVITY_SUN           (275.0F)                /**< The sun's gravity in m/s^2 */
        #define LSM303_GRAVITY_STANDARD      (LSM303_GRAVITY_EARTH)
        #define LSM303_ACCEL_MG_LSB          (0.001F)                // 1, 2, 4 or 12 mg per lsb

        typedef enum {                                          // DEFAULT    TYPE
            LSM303_REGISTER_ACCEL_CTRL_REG1_A         = 0x20,   // 00000111   rw
            LSM303_REGISTER_ACCEL_CTRL_REG2_A         = 0x21,   // 00000000   rw
            LSM303_REGISTER_ACCEL_CTRL_REG3_A         = 0x22,   // 00000000   rw
            LSM303_REGISTER_ACCEL_CTRL_REG4_A         = 0x23,   // 00000000   rw
            LSM303_REGISTER_ACCEL_CTRL_REG5_A         = 0x24,   // 00000000   rw
            LSM303_REGISTER_ACCEL_CTRL_REG6_A         = 0x25,   // 00000000   rw
            LSM303_REGISTER_ACCEL_REFERENCE_A         = 0x26,   // 00000000   r
            LSM303_REGISTER_ACCEL_STATUS_REG_A        = 0x27,   // 00000000   r
            LSM303_REGISTER_ACCEL_OUT_X_L_A           = 0x28,
            LSM303_REGISTER_ACCEL_OUT_X_H_A           = 0x29,
            LSM303_REGISTER_ACCEL_OUT_Y_L_A           = 0x2A,
            LSM303_REGISTER_ACCEL_OUT_Y_H_A           = 0x2B,
            LSM303_REGISTER_ACCEL_OUT_Z_L_A           = 0x2C,
            LSM303_REGISTER_ACCEL_OUT_Z_H_A           = 0x2D,
            LSM303_REGISTER_ACCEL_FIFO_CTRL_REG_A     = 0x2E,
            LSM303_REGISTER_ACCEL_FIFO_SRC_REG_A      = 0x2F,
            LSM303_REGISTER_ACCEL_INT1_CFG_A          = 0x30,
            LSM303_REGISTER_ACCEL_INT1_SOURCE_A       = 0x31,
            LSM303_REGISTER_ACCEL_INT1_THS_A          = 0x32,
            LSM303_REGISTER_ACCEL_INT1_DURATION_A     = 0x33,
            LSM303_REGISTER_ACCEL_INT2_CFG_A          = 0x34,
            LSM303_REGISTER_ACCEL_INT2_SOURCE_A       = 0x35,
            LSM303_REGISTER_ACCEL_INT2_THS_A          = 0x36,
            LSM303_REGISTER_ACCEL_INT2_DURATION_A     = 0x37,
            LSM303_REGISTER_ACCEL_CLICK_CFG_A         = 0x38,
            LSM303_REGISTER_ACCEL_CLICK_SRC_A         = 0x39,
            LSM303_REGISTER_ACCEL_CLICK_THS_A         = 0x3A,
            LSM303_REGISTER_ACCEL_TIME_LIMIT_A        = 0x3B,
            LSM303_REGISTER_ACCEL_TIME_LATENCY_A      = 0x3C,
            LSM303_REGISTER_ACCEL_TIME_WINDOW_A       = 0x3D
        } lsm303AccelRegisters_t;

        typedef struct lsm303AccelData_s
        {
            float x;
            float y;
            float z;
        } lsm303AccelData;

        /*
         * Private constructor. Object must be obtained through
         * constructor
         */
        lsm303Accelero() : i2c2_device(LSM303_ADDRESS_ACCEL)
        {
            // Enable the accelerometer (100Hz)
            writeReg(LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x57);
        }
        friend class SingletonTemplate<lsm303Accelero>;

    public:
        bool test();
        void getEvent(float *x, float *y, float *z);
};

class lsm303Magno : private i2c2_device, public SingletonTemplate<lsm303Magno>
{
    private:
        #define LSM303_ADDRESS_MAG            (0x3C)

        typedef enum {
            LSM303_REGISTER_MAG_CRA_REG_M             = 0x00,
            LSM303_REGISTER_MAG_CRB_REG_M             = 0x01,
            LSM303_REGISTER_MAG_MR_REG_M              = 0x02,
            LSM303_REGISTER_MAG_OUT_X_H_M             = 0x03,
            LSM303_REGISTER_MAG_OUT_X_L_M             = 0x04,
            LSM303_REGISTER_MAG_OUT_Z_H_M             = 0x05,
            LSM303_REGISTER_MAG_OUT_Z_L_M             = 0x06,
            LSM303_REGISTER_MAG_OUT_Y_H_M             = 0x07,
            LSM303_REGISTER_MAG_OUT_Y_L_M             = 0x08,
            LSM303_REGISTER_MAG_SR_REG_Mg             = 0x09,
            LSM303_REGISTER_MAG_IRA_REG_M             = 0x0A,
            LSM303_REGISTER_MAG_IRB_REG_M             = 0x0B,
            LSM303_REGISTER_MAG_IRC_REG_M             = 0x0C,
            LSM303_REGISTER_MAG_TEMP_OUT_H_M          = 0x31,
            LSM303_REGISTER_MAG_TEMP_OUT_L_M          = 0x32
        } lsm303MagRegisters_t;

        typedef enum
        {
            LSM303_MAGGAIN_1_3                        = 0x20,  // +/- 1.3
            LSM303_MAGGAIN_1_9                        = 0x40,  // +/- 1.9
            LSM303_MAGGAIN_2_5                        = 0x60,  // +/- 2.5
            LSM303_MAGGAIN_4_0                        = 0x80,  // +/- 4.0
            LSM303_MAGGAIN_4_7                        = 0xA0,  // +/- 4.7
            LSM303_MAGGAIN_5_6                        = 0xC0,  // +/- 5.6
            LSM303_MAGGAIN_8_1                        = 0xE0   // +/- 8.1
        } lsm303MagGain;

        lsm303Magno() :
            i2c2_device(LSM303_ADDRESS_MAG),
            _lsm303Accel_MG_LSB(0.001F),
            _lsm303Mag_Gauss_LSB_XY(1100.0F),
            _lsm303Mag_Gauss_LSB_Z(980.0F)
        {
            // Enable the magnetometer
            writeReg(LSM303_REGISTER_MAG_MR_REG_M, 0x00);
            setGain(LSM303_MAGGAIN_1_3);
            enableAutoRange(false);
        }
        friend class SingletonTemplate<lsm303Magno>;

    public:
        bool test();
        void getEvent(float *x, float *y, float *z, float *orientation = NULL);
        void enableAutoRange(bool enable)
        {
            _autoRangeEnabled = enable;
        }
        void setGain(lsm303MagGain gain);

    private:
        float           _lsm303Accel_MG_LSB;   // 1, 2, 4 or 12 mg per lsb
        float           _lsm303Mag_Gauss_LSB_XY;  // Varies with gain
        float           _lsm303Mag_Gauss_LSB_Z;   // Varies with gain
        lsm303MagGain   _magGain;
        bool            _autoRangeEnabled;
};

#endif /* LSM303_HPP_ */
