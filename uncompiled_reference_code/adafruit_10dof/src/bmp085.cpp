/*
 * bmp085.cpp
 *
 *  Created on: Jul 13, 2014
 *      Author: pardeep
 */

/***************************************************************************
 This is a library for the BMP085 pressure sensor

 Designed specifically to work with the Adafruit BMP085 or BMP180 Breakout
 ----> http://www.adafruit.com/products/391
 ----> http://www.adafruit.com/products/1603

 These displays use I2C to communicate, 2 pins are required to interface.

 Adafruit invests time and resources providing this open source code,
 please support Adafruit andopen-source hardware by purchasing products
 from Adafruit!

 Written by Kevin Townsend for Adafruit Industries.
 BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <math.h>
#include <limits.h>
#include <string.h>

#include "adafruit_10dof/bmp085.hpp"
#include "utilities.h"

#define BMP085_USE_DATASHEET_VALS (0) /* Set to 1 for sanity check */

/**************************************************************************/
/*!
 @brief  Reads the factory-set coefficients
 */
/**************************************************************************/
void bmp085::readCoefficients(void)
{
#if BMP085_USE_DATASHEET_VALS
    _bmp085_coeffs.ac1 = 408;
    _bmp085_coeffs.ac2 = -72;
    _bmp085_coeffs.ac3 = -14383;
    _bmp085_coeffs.ac4 = 32741;
    _bmp085_coeffs.ac5 = 32757;
    _bmp085_coeffs.ac6 = 23153;
    _bmp085_coeffs.b1 = 6190;
    _bmp085_coeffs.b2 = 4;
    _bmp085_coeffs.mb = -32768;
    _bmp085_coeffs.mc = -8711;
    _bmp085_coeffs.md = 2868;
    _bmp085Mode = 0;
#else
    _bmp085_coeffs.ac1 = (int16_t) get16BitRegister(BMP085_REGISTER_CAL_AC1);
    _bmp085_coeffs.ac2 = (int16_t) get16BitRegister(BMP085_REGISTER_CAL_AC2);
    _bmp085_coeffs.ac3 = (int16_t) get16BitRegister(BMP085_REGISTER_CAL_AC3);
    _bmp085_coeffs.ac4 = get16BitRegister(BMP085_REGISTER_CAL_AC4);
    _bmp085_coeffs.ac5 = get16BitRegister(BMP085_REGISTER_CAL_AC5);
    _bmp085_coeffs.ac6 = get16BitRegister(BMP085_REGISTER_CAL_AC6);
    _bmp085_coeffs.b1 = (int16_t) get16BitRegister(BMP085_REGISTER_CAL_B1);
    _bmp085_coeffs.b2 = (int16_t) get16BitRegister(BMP085_REGISTER_CAL_B2);
    _bmp085_coeffs.mb = (int16_t) get16BitRegister(BMP085_REGISTER_CAL_MB);
    _bmp085_coeffs.mc = (int16_t) get16BitRegister(BMP085_REGISTER_CAL_MC);
    _bmp085_coeffs.md = (int16_t) get16BitRegister(BMP085_REGISTER_CAL_MD);
#endif
}

/**************************************************************************/
/*!

 */
/**************************************************************************/
void bmp085::readRawTemperature(int32_t *temperature)
{
#if BMP085_USE_DATASHEET_VALS
    *temperature = 27898;
#else
    writeReg(BMP085_REGISTER_CONTROL, BMP085_REGISTER_READTEMPCMD);
    delay_ms(5);
    *temperature = get16BitRegister(BMP085_REGISTER_TEMPDATA);
#endif
}

/**************************************************************************/
/*!

 */
/**************************************************************************/
void bmp085::readRawPressure(int32_t *pressure)
{
#if BMP085_USE_DATASHEET_VALS
    *pressure = 23843;
#else
    uint8_t p8;
    uint16_t p16;
    int32_t p32;

    writeReg(BMP085_REGISTER_CONTROL,
            BMP085_REGISTER_READPRESSURECMD + (_bmp085Mode << 6));
    switch (_bmp085Mode)
    {
        case BMP085_MODE_ULTRALOWPOWER:
            delay_ms(5);
            break;
        case BMP085_MODE_STANDARD:
            delay_ms(8);
            break;
        case BMP085_MODE_HIGHRES:
            delay_ms(14);
            break;
        case BMP085_MODE_ULTRAHIGHRES:
        default:
            delay_ms(26);
            break;
    }

    p16 = get16BitRegister(BMP085_REGISTER_PRESSUREDATA);
    p32 = (uint32_t) p16 << 8;
    p8 = readReg(BMP085_REGISTER_PRESSUREDATA + 2);
    p32 += p8;
    p32 >>= (8 - _bmp085Mode);

    *pressure = p32;
#endif
}

/**************************************************************************/
/*!
 @brief  Compute B5 coefficient used in temperature & pressure calcs.
 */
/**************************************************************************/
int32_t bmp085::computeB5(int32_t ut)
{
    int32_t X1 = (ut - (int32_t) _bmp085_coeffs.ac6)
            * ((int32_t) _bmp085_coeffs.ac5) >> 15;
    int32_t X2 = ((int32_t) _bmp085_coeffs.mc << 11)
            / (X1 + (int32_t) _bmp085_coeffs.md);
    return X1 + X2;
}

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/

/**************************************************************************/
/*!
 @brief  Instantiates a new Adafruit_BMP085_Unified class
 */
/**************************************************************************/

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/
#include <stdio.h>

bool bmp085::test()
{
    return (0x55 == readReg(BMP085_REGISTER_CHIPID));
}

/**************************************************************************/
/*!
 @brief  Gets the compensated pressure level in kPa
 */
/**************************************************************************/
void bmp085::getPressure(float *pressure)
{
    int32_t ut = 0, up = 0, compp = 0;
    int32_t x1, x2, b5, b6, x3, b3, p;
    uint32_t b4, b7;

    /* Get the raw pressure and temperature values */
    readRawTemperature(&ut);
    readRawPressure(&up);

    /* Temperature compensation */
    b5 = computeB5(ut);

    /* Pressure compensation */
    b6 = b5 - 4000;
    x1 = (_bmp085_coeffs.b2 * ((b6 * b6) >> 12)) >> 11;
    x2 = (_bmp085_coeffs.ac2 * b6) >> 11;
    x3 = x1 + x2;
    b3 = (((((int32_t) _bmp085_coeffs.ac1) * 4 + x3) << _bmp085Mode) + 2) >> 2;
    x1 = (_bmp085_coeffs.ac3 * b6) >> 13;
    x2 = (_bmp085_coeffs.b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (_bmp085_coeffs.ac4 * (uint32_t) (x3 + 32768)) >> 15;
    b7 = ((uint32_t) (up - b3) * (50000 >> _bmp085Mode));

    if (b7 < 0x80000000)
    {
        p = (b7 << 1) / b4;
    }
    else
    {
        p = (b7 / b4) << 1;
    }

    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    compp = p + ((x1 + x2 + 3791) >> 4);

    /* Assign compensated pressure value */
    *pressure = compp;
}

/**************************************************************************/
/*!
 @brief  Reads the temperatures in degrees Celsius
 */
/**************************************************************************/
void bmp085::getTemperature(float *temp)
{
    int32_t UT, X1, X2, B5; // following ds convention
    float t;

    readRawTemperature(&UT);

#if BMP085_USE_DATASHEET_VALS
    // use datasheet numbers!
    UT = 27898;
    _bmp085_coeffs.ac6 = 23153;
    _bmp085_coeffs.ac5 = 32757;
    _bmp085_coeffs.mc = -8711;
    _bmp085_coeffs.md = 2868;
#endif

    B5 = computeB5(UT);
    t = (B5 + 8) >> 4;
    t /= 10;

    *temp = t;
}

/**************************************************************************/
/*!
 Calculates the altitude (in meters) from the specified atmospheric
 pressure (in hPa), sea-level pressure (in hPa), and temperature (in �C)

 @param  seaLevel      Sea-level pressure in hPa
 @param  atmospheric   Atmospheric pressure in hPa
 @param  temp          Temperature in degrees Celsius
 */
/**************************************************************************/
float bmp085::pressureToAltitude(float seaLevel, float atmospheric, float temp)
{
    /* Hyposometric formula:                      */
    /*                                            */
    /*     ((P0/P)^(1/5.257) - 1) * (T + 273.15)  */
    /* h = -------------------------------------  */
    /*                   0.0065                   */
    /*                                            */
    /* where: h   = height (in meters)            */
    /*        P0  = sea-level pressure (in hPa)   */
    /*        P   = atmospheric pressure (in hPa) */
    /*        T   = temperature (in �C)           */

    return (((float) pow((seaLevel / atmospheric), 0.190223F) - 1.0F)
            * (temp + 273.15F)) / 0.0065F;
}

float bmp085::seaLevelForAltitude(float altitude, float atmospheric, float temp)
{
    /* Hyposometric formula:                      */
    /*                                            */
    /* P0=((((h*0.0065)/(temp + 273.15F))+1)^(^/0.190223F))*P */
    /*                                            */
    /* where: h   = height (in meters)            */
    /*        P0  = sea-level pressure (in hPa)   */
    /*        P   = atmospheric pressure (in hPa) */
    /*        T   = temperature (in �C)           */

    return (float) pow((((altitude * 0.0065) / (temp + 273.15F)) + 1),
            (1.0 / 0.190223F)) * atmospheric;
}

/**************************************************************************/
/*!
 @brief  Reads the sensor and returns the data as a sensors_event_t
 */
/**************************************************************************/
float bmp085::getEvent(void)
{
    float pressure_kPa;

    getPressure(&pressure_kPa);
    return (pressure_kPa / 100.0F);
}

