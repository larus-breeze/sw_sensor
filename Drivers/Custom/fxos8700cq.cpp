#include "fxos8700cq.h"
#include "my_assert.h"
#include "FreeRTOS.h"
#include "FreeRTOS_wrapper.h"
/*
 * Definitions taken from https://github.com/adafruit/Adafruit_FXOS8700/blob/master/Adafruit_FXOS8700.cpp
 */

/** Macro for mg per LSB at +/- 2g sensitivity (1 LSB = 0.000244mg) */
#define ACCEL_MG_LSB_2G (0.000244F)
/** Macro for mg per LSB at +/- 4g sensitivity (1 LSB = 0.000488mg) */
#define ACCEL_MG_LSB_4G (0.000488F)
/** Macro for mg per LSB at +/- 8g sensitivity (1 LSB = 0.000976mg) */
#define ACCEL_MG_LSB_8G (0.000976F)
/** Macro for micro tesla (uT) per LSB (1 LSB = 0.1uT) */
#define MAG_UT_LSB (0.1F)

#define ACCEL_GRAVITY_MS2 (9.81)
#define FXOS8700_ID (0xC7) // 1100 0111

#define FXOS8700_I2C  &hi2c1
#define FXOS8700_I2C_ADDR 0x3C

bool FXOS8700_write(uint8_t reg, uint8_t *tx_data, uint8_t length)
{
	if(I2C_OK == I2C_WriteRegister(FXOS8700_I2C, FXOS8700_I2C_ADDR, reg, 1, tx_data, length))
	{
		return true;
	}
	return false;
}

bool FXOS8700_read(uint8_t reg, uint8_t *rx_data, uint8_t length)
{
	if(I2C_OK == I2C_ReadRegister(FXOS8700_I2C, FXOS8700_I2C_ADDR, reg, 1, rx_data, length))
	{
		return true;
	}
	return false;
}

COMMON static uint8_t FXOS8700_range = 0;   /* instance variable*/

bool FXOS8700_Initialize(fxos8700AccelRange_t range)
{
	uint8_t id = 0;
	if(false == FXOS8700_read(FXOS8700_REGISTER_WHO_AM_I, &id, 1))
		return false;
	ASSERT(FXOS8700_ID == id);

	FXOS8700_range = range;

	uint8_t reg_data = 0;

	/* Set to standby mode (required to make changes to this register) */
	reg_data = 0;
	if(false == FXOS8700_write(FXOS8700_REGISTER_CTRL_REG1, &reg_data, 1))
		return false;

	reg_data = (uint8_t) range;
	ASSERT((reg_data <= ACCEL_RANGE_8G) && (reg_data >= ACCEL_RANGE_2G));
	if(false == FXOS8700_write(FXOS8700_REGISTER_XYZ_DATA_CFG, &reg_data, 1))
		return false;

	/* High resolution */
	reg_data = 0x02;
	if(false == FXOS8700_write(FXOS8700_REGISTER_CTRL_REG2, &reg_data, 1))
		return false;

	/* Active, Normal Mode, Low Noise, 100Hz in Hybrid Mode */
	reg_data = 0x15;
	if(false == FXOS8700_write(FXOS8700_REGISTER_CTRL_REG1, &reg_data, 1))
		return false;
	/* Configure the magnetometer */
	/* Hybrid Mode, Over Sampling Rate = 16 */
	reg_data = 0x1F;
	if(false == FXOS8700_write(FXOS8700_REGISTER_MCTRL_REG1, &reg_data, 1))
		return false;

	/* Jump to reg 0x33 after reading 0x06, to get acc and mag data in one read */
	reg_data = 0x20;
	if(false == FXOS8700_write(FXOS8700_REGISTER_MCTRL_REG2, &reg_data, 1))
		return false;

	return true;
}

#define RX_DATA_SIZE 13
COMMON static uint8_t rx_data[RX_DATA_SIZE];
bool FXOS8700_get(float* xyz_acc, float* xyz_mag)
{

	if(true == FXOS8700_read(FXOS8700_REGISTER_STATUS, rx_data, RX_DATA_SIZE))
	{
		/*TODO: Evaluate rx_data[0] status values might activate interrupts for new conversion done trigger*/

		float acceleration_conversion_factor = 0;
		if (ACCEL_RANGE_2G == FXOS8700_range)
		{
			acceleration_conversion_factor = ACCEL_MG_LSB_2G * ACCEL_GRAVITY_MS2;
		}
		else if (ACCEL_RANGE_4G == FXOS8700_range)
		{
			acceleration_conversion_factor = ACCEL_MG_LSB_4G * ACCEL_GRAVITY_MS2;
		}
		else if (ACCEL_RANGE_8G == FXOS8700_range)
		{
			acceleration_conversion_factor = ACCEL_MG_LSB_8G * ACCEL_GRAVITY_MS2;
		}
		else
		{
			ASSERT(0);
		}

		for(int i = 0; i<3; i++)
		{
			xyz_acc[i] = (float)((int16_t)((rx_data[(i*2)+1] << 8)  | (rx_data[(i*2)+2])) >> 2) * acceleration_conversion_factor ;
			xyz_mag[i] = ((float)(int16_t)((rx_data[(i*2)+7] << 8)  | rx_data[(i*2)+8])) * MAG_UT_LSB;
		}
		return true;
	}
	return false;
}
