#ifndef __FXOS8700_H
#define __FXOS8700_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "i2c.h"

/*
 *  Register definition taken from: https://github.com/adafruit/Adafruit_FXOS8700/blob/master/Adafruit_FXOS8700.h
 */
/*!
    Raw register addresses used to communicate with the sensor.
*/
typedef enum {
  FXOS8700_REGISTER_STATUS = 0x00,    /**< 0x00 */
  FXOS8700_REGISTER_OUT_X_MSB = 0x01, /**< 0x01 */
  FXOS8700_REGISTER_OUT_X_LSB = 0x02, /**< 0x02 */
  FXOS8700_REGISTER_OUT_Y_MSB = 0x03, /**< 0x03 */
  FXOS8700_REGISTER_OUT_Y_LSB = 0x04, /**< 0x04 */
  FXOS8700_REGISTER_OUT_Z_MSB = 0x05, /**< 0x05 */
  FXOS8700_REGISTER_OUT_Z_LSB = 0x06, /**< 0x06 */
  FXOS8700_REGISTER_WHO_AM_I =
      0x0D, /**< 0x0D (default value = 0b11000111, read only) */
  FXOS8700_REGISTER_XYZ_DATA_CFG = 0x0E, /**< 0x0E */
  FXOS8700_REGISTER_CTRL_REG1 =
      0x2A, /**< 0x2A (default value = 0b00000000, read/write) */
  FXOS8700_REGISTER_CTRL_REG2 =
      0x2B, /**< 0x2B (default value = 0b00000000, read/write) */
  FXOS8700_REGISTER_CTRL_REG3 =
      0x2C, /**< 0x2C (default value = 0b00000000, read/write) */
  FXOS8700_REGISTER_CTRL_REG4 =
      0x2D, /**< 0x2D (default value = 0b00000000, read/write) */
  FXOS8700_REGISTER_CTRL_REG5 =
      0x2E, /**< 0x2E (default value = 0b00000000, read/write) */
  FXOS8700_REGISTER_MSTATUS = 0x32,    /**< 0x32 */
  FXOS8700_REGISTER_MOUT_X_MSB = 0x33, /**< 0x33 */
  FXOS8700_REGISTER_MOUT_X_LSB = 0x34, /**< 0x34 */
  FXOS8700_REGISTER_MOUT_Y_MSB = 0x35, /**< 0x35 */
  FXOS8700_REGISTER_MOUT_Y_LSB = 0x36, /**< 0x36 */
  FXOS8700_REGISTER_MOUT_Z_MSB = 0x37, /**< 0x37 */
  FXOS8700_REGISTER_MOUT_Z_LSB = 0x38, /**< 0x38 */
  FXOS8700_REGISTER_MCTRL_REG1 =
      0x5B, /**< 0x5B (default value = 0b00000000, read/write) */
  FXOS8700_REGISTER_MCTRL_REG2 =
      0x5C, /**< 0x5C (default value = 0b00000000, read/write) */
  FXOS8700_REGISTER_MCTRL_REG3 =
      0x5D, /**< 0x5D (default value = 0b00000000, read/write) */
} fxos8700Registers_t;
/*=========================================================================*/

/*=========================================================================
    OPTIONAL SPEED SETTINGS
    -----------------------------------------------------------------------*/
/*!
    Range settings for the accelerometer sensor.
*/
typedef enum {
  ACCEL_RANGE_2G = 0x00, /**< +/- 2g range */
  ACCEL_RANGE_4G = 0x01, /**< +/- 4g range */
  ACCEL_RANGE_8G = 0x02  /**< +/- 8g range */
} fxos8700AccelRange_t;
/*=========================================================================*/

/*=========================================================================
    RAW GYROSCOPE DATA TYPE
    -----------------------------------------------------------------------*/
/*!
    @brief  Raw (integer) values from the gyroscope sensor.
*/
typedef struct {
  int16_t x; /**< Raw int16_t value from the x axis */
  int16_t y; /**< Raw int16_t value from the y axis */
  int16_t z; /**< Raw int16_t value from the z axis */
} fxos8700RawData_t;


bool FXOS8700_Initialize(fxos8700AccelRange_t range);
bool FXOS8700_get(float* xyz_acc, float* xyz_mag);

#ifdef __cplusplus
}
#endif

#endif /* __FXOS8700_H */
