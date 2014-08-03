/*
 * lsm303.cpp
 *
 *  Created on: Jul 20, 2014
 *      Author: pardeep
 */


#include "adafruit_10dof/lsm303.hpp"


bool lsm303Accelero::test()
{
    return (0x57 == readReg(LSM303_REGISTER_ACCEL_CTRL_REG1_A));
}

void lsm303Accelero::getEvent(float *x, float *y, float *z)
{
    uint8_t xlo = readReg(LSM303_REGISTER_ACCEL_OUT_X_L_A);
    uint8_t xhi = readReg(LSM303_REGISTER_ACCEL_OUT_X_H_A);
    uint8_t ylo = readReg(LSM303_REGISTER_ACCEL_OUT_Y_L_A);
    uint8_t yhi = readReg(LSM303_REGISTER_ACCEL_OUT_Y_H_A);
    uint8_t zlo = readReg(LSM303_REGISTER_ACCEL_OUT_Z_L_A);
    uint8_t zhi = readReg(LSM303_REGISTER_ACCEL_OUT_Z_H_A);

    // Shift values to create properly formed integer (low byte first)
    *x = (int16_t)(xlo | (xhi << 8)) >> 4;
    *y = (int16_t)(ylo | (yhi << 8)) >> 4;
    *z = (int16_t)(zlo | (zhi << 8)) >> 4;

    *x = *x * LSM303_ACCEL_MG_LSB * LSM303_GRAVITY_STANDARD;
    *y = *y * LSM303_ACCEL_MG_LSB * LSM303_GRAVITY_STANDARD;
    *z = *z * LSM303_ACCEL_MG_LSB * LSM303_GRAVITY_STANDARD;
}

void lsm303Magno::setGain(lsm303MagGain gain)
{
    writeReg(LSM303_REGISTER_MAG_CRB_REG_M, (uint8_t) gain);
    _magGain = gain;
    switch (gain)
    {
        case LSM303_MAGGAIN_1_3:
            _lsm303Mag_Gauss_LSB_XY = 1100;
            _lsm303Mag_Gauss_LSB_Z = 980;
            break;
        case LSM303_MAGGAIN_1_9:
            _lsm303Mag_Gauss_LSB_XY = 855;
            _lsm303Mag_Gauss_LSB_Z = 760;
            break;
        case LSM303_MAGGAIN_2_5:
            _lsm303Mag_Gauss_LSB_XY = 670;
            _lsm303Mag_Gauss_LSB_Z = 600;
            break;
        case LSM303_MAGGAIN_4_0:
            _lsm303Mag_Gauss_LSB_XY = 450;
            _lsm303Mag_Gauss_LSB_Z = 400;
            break;
        case LSM303_MAGGAIN_4_7:
            _lsm303Mag_Gauss_LSB_XY = 400;
            _lsm303Mag_Gauss_LSB_Z = 255;
            break;
        case LSM303_MAGGAIN_5_6:
            _lsm303Mag_Gauss_LSB_XY = 330;
            _lsm303Mag_Gauss_LSB_Z = 295;
            break;
        case LSM303_MAGGAIN_8_1:
            _lsm303Mag_Gauss_LSB_XY = 230;
            _lsm303Mag_Gauss_LSB_Z = 205;
            break;
    }
}

bool lsm303Magno::test()
{
    /*
     * LSM303DLHC has no WHOAMI register so read CRA_REG_M to check
     * the default value (0b00010000/0x10)
     */
    return (0x10 == lsm303Magno::getInstance().readReg(LSM303_REGISTER_MAG_CRA_REG_M));
}

void lsm303Magno::getEvent(float *x, float *y, float *z, float *orientation)
{
    bool readingValid = false;

    while (!readingValid)
    {
        uint8_t xhi = readReg(LSM303_REGISTER_MAG_OUT_X_H_M);
        uint8_t xlo = readReg(LSM303_REGISTER_MAG_OUT_X_L_M);
        uint8_t zhi = readReg(LSM303_REGISTER_MAG_OUT_Z_H_M);
        uint8_t zlo = readReg(LSM303_REGISTER_MAG_OUT_Z_L_M);
        uint8_t yhi = readReg(LSM303_REGISTER_MAG_OUT_Y_H_M);
        uint8_t ylo = readReg(LSM303_REGISTER_MAG_OUT_Y_L_M);

        // Shift values to create properly formed integer (low byte first)
        *x = (int16_t) (xlo | ((int16_t) xhi << 8));
        *y = (int16_t) (ylo | ((int16_t) yhi << 8));
        *z = (int16_t) (zlo | ((int16_t) zhi << 8));

        // ToDo: Calculate orientation
        if (orientation) {
            *orientation = 0.0;
        }
        /* Make sure the sensor isn't saturating if auto-ranging is enabled */
        if (!_autoRangeEnabled)
        {
            readingValid = true;
        }
        else
        {
            /* Check if the sensor is saturating or not */
            if ((*x >= 4090) || (*x <= -4090) ||
                (*y >= 4090) || (*y <= -4090)  ||
                (*z >= 4090) || (*z <= -4090)) {
                /* Saturating .... increase the range if we can */
                switch (_magGain) {
                    case LSM303_MAGGAIN_5_6:
                        setGain(LSM303_MAGGAIN_8_1);
                        readingValid = false;
                        break;
                    case LSM303_MAGGAIN_4_7:
                        setGain(LSM303_MAGGAIN_5_6);
                        readingValid = false;
                        break;
                    case LSM303_MAGGAIN_4_0:
                        setGain(LSM303_MAGGAIN_4_7);
                        readingValid = false;
                        break;
                    case LSM303_MAGGAIN_2_5:
                        setGain(LSM303_MAGGAIN_4_0);
                        readingValid = false;
                        break;
                    case LSM303_MAGGAIN_1_9:
                        setGain(LSM303_MAGGAIN_2_5);
                        readingValid = false;
                        break;
                    case LSM303_MAGGAIN_1_3:
                        setGain(LSM303_MAGGAIN_1_9);
                        readingValid = false;
                        break;
                    default:
                        readingValid = true;
                        break;
                }
            }
            else {
                /* All values are withing range */
                readingValid = true;
            }
        }
    }
}
