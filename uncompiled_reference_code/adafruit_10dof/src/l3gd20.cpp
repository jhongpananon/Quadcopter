/*
 * l3gd20.cpp
 *
 *  Created on: Jul 13, 2014
 *      Author: pardeep
 */


#include "adafruit_10dof/l3gd20.hpp"
#include <stdio.h>

l3gd20::l3gd20() :
    i2c2_device(L3GD20_ADDRESS),
    _range(GYRO_RANGE_250DPS),
    _autoRangeEnabled(false)
{
    /* Set CTRL_REG1 (0x20)
     ====================================================================
     BIT  Symbol    Description                                   Default
     ---  ------    --------------------------------------------- -------
     7-6  DR1/0     Output data rate                                   00
     5-4  BW1/0     Bandwidth selection                                00
       3  PD        0 = Power-down mode, 1 = normal/sleep mode          0
       2  ZEN       Z-axis enable (0 = disabled, 1 = enabled)           1
       1  YEN       Y-axis enable (0 = disabled, 1 = enabled)           1
       0  XEN       X-axis enable (0 = disabled, 1 = enabled)           1 */

    /* Reset then switch to normal mode and enable all three channels */
    writeReg(GYRO_REGISTER_CTRL_REG1, 0x00);
    writeReg(GYRO_REGISTER_CTRL_REG1, 0x0F);
    /* ------------------------------------------------------------------ */

    /* Set CTRL_REG2 (0x21)
     ====================================================================
     BIT  Symbol    Description                                   Default
     ---  ------    --------------------------------------------- -------
     5-4  HPM1/0    High-pass filter mode selection                    00
     3-0  HPCF3..0  High-pass filter cutoff frequency selection      0000 */

    /* Nothing to do ... keep default values */
    /* ------------------------------------------------------------------ */

    /* Set CTRL_REG3 (0x22)
     ====================================================================
     BIT  Symbol    Description                                   Default
     ---  ------    --------------------------------------------- -------
       7  I1_Int1   Interrupt enable on INT1 (0=disable,1=enable)       0
       6  I1_Boot   Boot status on INT1 (0=disable,1=enable)            0
       5  H-Lactive Interrupt active config on INT1 (0=high,1=low)      0
       4  PP_OD     Push-Pull/Open-Drain (0=PP, 1=OD)                   0
       3  I2_DRDY   Data ready on DRDY/INT2 (0=disable,1=enable)        0
       2  I2_WTM    FIFO wtrmrk int on DRDY/INT2 (0=dsbl,1=enbl)        0
       1  I2_ORun   FIFO overrun int on DRDY/INT2 (0=dsbl,1=enbl)       0
       0  I2_Empty  FIFI empty int on DRDY/INT2 (0=dsbl,1=enbl)         0 */

    /* Nothing to do ... keep default values */
    /* ------------------------------------------------------------------ */

    /* Set CTRL_REG4 (0x23)
     ====================================================================
     BIT  Symbol    Description                                   Default
     ---  ------    --------------------------------------------- -------
       7  BDU       Block Data Update (0=continuous, 1=LSB/MSB)         0
       6  BLE       Big/Little-Endian (0=Data LSB, 1=Data MSB)          0
     5-4  FS1/0     Full scale selection                               00
                                    00 = 250 dps
                                    01 = 500 dps
                                    10 = 2000 dps
                                    11 = 2000 dps
       0  SIM       SPI Mode (0=4-wire, 1=3-wire)                       0 */

    /* Adjust resolution if requested */
    switch(_range)
    {
      case GYRO_RANGE_250DPS:
        writeReg(GYRO_REGISTER_CTRL_REG4, 0x00);
        break;
      case GYRO_RANGE_500DPS:
        writeReg(GYRO_REGISTER_CTRL_REG4, 0x10);
        break;
      case GYRO_RANGE_2000DPS:
        writeReg(GYRO_REGISTER_CTRL_REG4, 0x20);
        break;
    }
    /* ------------------------------------------------------------------ */

    /* Set CTRL_REG5 (0x24)
     ====================================================================
     BIT  Symbol    Description                                   Default
     ---  ------    --------------------------------------------- -------
       7  BOOT      Reboot memory content (0=normal, 1=reboot)          0
       6  FIFO_EN   FIFO enable (0=FIFO disable, 1=enable)              0
       4  HPen      High-pass filter enable (0=disable,1=enable)        0
     3-2  INT1_SEL  INT1 Selection config                              00
     1-0  OUT_SEL   Out selection config                               00 */

    /* Nothing to do ... keep default values */
    /* ------------------------------------------------------------------ */
}

bool l3gd20::test()
{
    /*
     * Make sure we have the correct chip ID since this checks
     * for correct address and that the IC is properly connected
     */
    return (L3GD20_ID == readReg(GYRO_REGISTER_WHO_AM_I));
}

void l3gd20::enableAutoRange(bool enable)
{
    _autoRangeEnabled = enable;
}

void l3gd20::getEvent(float *x, float *y, float *z)
{
    bool readingValid = false;

    while(!readingValid)
    {
        uint8_t xlo = readReg(GYRO_REGISTER_OUT_X_L);
        uint8_t xhi = readReg(GYRO_REGISTER_OUT_X_H);
        uint8_t ylo = readReg(GYRO_REGISTER_OUT_Y_L);
        uint8_t yhi = readReg(GYRO_REGISTER_OUT_Y_H);
        uint8_t zlo = readReg(GYRO_REGISTER_OUT_Z_L);
        uint8_t zhi = readReg(GYRO_REGISTER_OUT_Z_H);

//        printf("[%x], [%x], [%x], [%x], [%x], [%x]\n",
//               xlo, xhi, ylo, yhi, zlo, zhi);
        /* Shift values to create properly formed integer (low byte first) */
        *x = (int16_t)(xlo | (xhi << 8));
        *y = (int16_t)(ylo | (yhi << 8));
        *z = (int16_t)(zlo | (zhi << 8));

        /* Make sure the sensor isn't saturating if auto-ranging is enabled */
        if (!_autoRangeEnabled) {
            readingValid = true;
        } else {
            /* Check if the sensor is saturating or not */
            if ( (*x >= 32760) || (*x <= -32760) ||
                 (*y >= 32760) || (*y <= -32760) ||
                 (*z >= 32760) ||  (*z <= -32760) ) {
                /* Saturating .... increase the range if we can */
                switch(_range)
                {
                    case GYRO_RANGE_500DPS:
                        /* Push the range up to 2000dps */
                        _range = GYRO_RANGE_2000DPS;
                        writeReg(GYRO_REGISTER_CTRL_REG1, 0x00);
                        writeReg(GYRO_REGISTER_CTRL_REG1, 0x0F);
                        writeReg(GYRO_REGISTER_CTRL_REG4, 0x20);
                        writeReg(GYRO_REGISTER_CTRL_REG5, 0x80);
                        readingValid = false;
                        break;
                    case GYRO_RANGE_250DPS:
                        /* Push the range up to 500dps */
                        _range = GYRO_RANGE_500DPS;
                        writeReg(GYRO_REGISTER_CTRL_REG1, 0x00);
                        writeReg(GYRO_REGISTER_CTRL_REG1, 0x0F);
                        writeReg(GYRO_REGISTER_CTRL_REG4, 0x10);
                        writeReg(GYRO_REGISTER_CTRL_REG5, 0x80);
                        readingValid = false;
                        break;
                    default:
                        readingValid = true;
                        break;
                }
            } else {
              /* All values are withing range */
              readingValid = true;
            }
        }
    }

    /* Compensate values depending on the resolution */
    switch(_range)
    {
      case GYRO_RANGE_250DPS:
        *x *= GYRO_SENSITIVITY_250DPS;
        *y *= GYRO_SENSITIVITY_250DPS;
        *z *= GYRO_SENSITIVITY_250DPS;
        break;
      case GYRO_RANGE_500DPS:
        *x *= GYRO_SENSITIVITY_500DPS;
        *y *= GYRO_SENSITIVITY_500DPS;
        *z *= GYRO_SENSITIVITY_500DPS;
        break;
      case GYRO_RANGE_2000DPS:
        *x *= GYRO_SENSITIVITY_2000DPS;
        *y *= GYRO_SENSITIVITY_2000DPS;
        *z *= GYRO_SENSITIVITY_2000DPS;
        break;
    }

    /* Convert values to rad/s */
    *x *= SENSORS_DPS_TO_RADS;
    *y *= SENSORS_DPS_TO_RADS;
    *z *= SENSORS_DPS_TO_RADS;
}
