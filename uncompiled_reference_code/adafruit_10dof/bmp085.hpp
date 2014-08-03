/*
 * bmp085.hpp
 *
 *  Created on: Jul 13, 2014
 *      Author: pardeep
 */

#ifndef BMP085_HPP_
#define BMP085_HPP_

#include "i2c2_device.hpp"
#include <string.h>

#define BMP085_ADDRESS                (0xEE)

class bmp085: private i2c2_device, public SingletonTemplate<bmp085>
{
    private:
        enum
        {
            BMP085_REGISTER_CAL_AC1         = 0xAA, // R   Calibration data (16 bits)
            BMP085_REGISTER_CAL_AC2         = 0xAC, // R   Calibration data (16 bits)
            BMP085_REGISTER_CAL_AC3         = 0xAE, // R   Calibration data (16 bits)
            BMP085_REGISTER_CAL_AC4         = 0xB0, // R   Calibration data (16 bits)
            BMP085_REGISTER_CAL_AC5         = 0xB2, // R   Calibration data (16 bits)
            BMP085_REGISTER_CAL_AC6         = 0xB4, // R   Calibration data (16 bits)
            BMP085_REGISTER_CAL_B1          = 0xB6, // R   Calibration data (16 bits)
            BMP085_REGISTER_CAL_B2          = 0xB8, // R   Calibration data (16 bits)
            BMP085_REGISTER_CAL_MB          = 0xBA, // R   Calibration data (16 bits)
            BMP085_REGISTER_CAL_MC          = 0xBC, // R   Calibration data (16 bits)
            BMP085_REGISTER_CAL_MD          = 0xBE, // R   Calibration data (16 bits)
            BMP085_REGISTER_CHIPID          = 0xD0,
            BMP085_REGISTER_VERSION         = 0xD1,
            BMP085_REGISTER_SOFTRESET       = 0xE0,
            BMP085_REGISTER_CONTROL         = 0xF4,
            BMP085_REGISTER_TEMPDATA        = 0xF6,
            BMP085_REGISTER_PRESSUREDATA    = 0xF6,
            BMP085_REGISTER_READTEMPCMD     = 0x2E,
            BMP085_REGISTER_READPRESSURECMD = 0x34
        };

        typedef enum
        {
            BMP085_MODE_ULTRALOWPOWER = 0,
            BMP085_MODE_STANDARD      = 1,
            BMP085_MODE_HIGHRES       = 2,
            BMP085_MODE_ULTRAHIGHRES  = 3
        } bmp085_mode_t;

        typedef struct
        {
            int16_t     ac1;
            int16_t     ac2;
            int16_t     ac3;
            uint16_t    ac4;
            uint16_t    ac5;
            uint16_t    ac6;
            int16_t     b1;
            int16_t     b2;
            int16_t     mb;
            int16_t     mc;
            int16_t     md;
        } bmp085_calib_data;

        bmp085() : i2c2_device(BMP085_ADDRESS)
        {
            _bmp085Mode = BMP085_MODE_ULTRAHIGHRES;
            /* Coefficients need to be read once */
            readCoefficients();
        }
        friend class SingletonTemplate<bmp085>;

    public:
        bool test();
        void getTemperature(float *temp);
        void getPressure(float *pressure);
        float pressureToAltitude(float seaLevel, float atmospheric, float temp);
        float seaLevelForAltitude(float altitude, float atmospheric, float temp);
        float getEvent(void);

    private:

        int32_t             computeB5(int32_t ut);
        bmp085_calib_data   _bmp085_coeffs;
        uint8_t             _bmp085Mode;

        void readCoefficients(void);
        void readRawTemperature(int32_t *temperature);
        void readRawPressure(int32_t *pressure);
};

#endif /* BMP085_HPP_ */
