#include "system_configuration.h"
#include <ms5611.h>
#include "FreeRTOS.h"
#include "task.h"
#include "my_assert.h"
#include "i2c.h"
#include "main.h"

#if RUN_MS5611_MODULE == 1

/* Address of ms5611 device- LSB can be set to 1 / 0 - thus two of these devices can be used */
// 0xEC means CSB (bit0) of the adress is connected to vcc   0xEE if connected to gnd //
#define MS5611_I2C &hi2c2

/* Ms5611 register locations */
#define CMD_RESET               0x1E // ADC reset command
#define CMD_ADC_READ            0x00 // ADC read command
#define CMD_ADC_CONV            0x40 // ADC conversion command

#define CMD_ADC_D1              0x00 // ADC D1 conversion
#define CMD_ADC_D2              0x10 // ADC D2 conversion

#define CMD_ADC_256             0x00 // ADC OSR=256
#define CMD_ADC_512             0x02 // ADC OSR=512
#define CMD_ADC_1024            0x04 // ADC OSR=1024
#define CMD_ADC_2048            0x06 // ADC OSR=2048
#define CMD_ADC_4096            0x08 // ADC OSR=4096 PATCH

#define CMD_PROM_RD             0xA0 // Prom read command

void MS5611::start_pressure_conversion(void) {
	uint8_t data = CMD_ADC_CONV | CMD_ADC_D1 | CMD_ADC_4096;
	I2C_Write(MS5611_I2C, I2C_address, &data, 1);
}

void MS5611::start_temperature_conversion(void) {
	uint8_t data = CMD_ADC_CONV | CMD_ADC_D2 | CMD_ADC_4096;
	I2C_Write(MS5611_I2C, I2C_address, &data, 1);
}
inline uint8_t MS5611::get_crc4() {
	int32_t cnt;
	uint32_t n_rem;
	uint16_t crc_read;
	uint8_t n_bit;

	n_rem = 0x00;
	crc_read = PromData[7]; // Save your crc into crc_read for later restauration;
	PromData[7] = (0xFF00 & PromData[7]); // Byte containing CRC is replaced by 0

	for (cnt = 0; cnt < 16; cnt++) {
		if (cnt % 2 == 1)
			n_rem ^= (uint16_t) ((PromData[cnt >> 1]) & 0x00FF);
		else
			n_rem ^= (uint16_t) (PromData[cnt >> 1] >> 8);

		for (n_bit = 8; n_bit > 0; n_bit--) {
			if (n_rem & (0x8000))
				n_rem = (n_rem << 1) ^ 0x3000;
			else
				n_rem = (n_rem << 1);
		}
	}

	n_rem = (0x000F & (n_rem >> 12));
	PromData[7] = crc_read;
	return (uint8_t) n_rem;		// Final CRC
}

inline uint32_t MS5611::read_24_bits(void) {
	static const uint8_t RX_BUFLEN = 3;

	uint8_t Buffer_Rx[RX_BUFLEN];
	uint32_t data = 0;

	I2C_ReadRegister(MS5611_I2C, I2C_address, CMD_ADC_READ, 1, Buffer_Rx,
			RX_BUFLEN);

	for (uint8_t i = 0; i < RX_BUFLEN; i++)
		data = (data << 8) + Buffer_Rx[i];

	return data;
}

inline void MS5611::calibrate(const uint32_t d1, const uint32_t d2) {
	int64_t D1 = d1;
	int64_t D2 = d2;
	int64_t celsius;
	int64_t dT; //difference between actual and measured temp.
	int64_t OFF; //offset at actual celsius
	int64_t SENS; //sensitivity at actual celsius
	int64_t T2;
	int64_t OFF2;
	int64_t SENS2;
	int64_t TEMP;
	int64_t temp1;
	int64_t temp2;

	dT = D2 - ((uint32_t) (PromData[5]) << 8);

	temp1 = (int64_t) (PromData[2]) << 16;
	temp2 = (dT * PromData[4]) >> 7;

	OFF = temp1 + temp2;

	SENS = ((int64_t) (PromData[1]) << 15) + (dT * (PromData[3]) >> 8);

	celsius = (dT * PromData[6]) >> 23;
	celsius += 2000;

	//SECOND ORDER TEMP COMPENSATION
	if (celsius < 2000) {

		T2 = (dT * dT) >> 31;
		TEMP = celsius - 2000;
		OFF2 = (5 * TEMP * TEMP) >> 1;
		SENS2 = (5 * TEMP * TEMP) >> 2;

		if (celsius < -1500) {
			TEMP = celsius + 1500;
			OFF2 = OFF2 + 7 * TEMP * TEMP;
			SENS2 = SENS2 + (((11 * TEMP * TEMP) >> 1));
		}

		celsius -= T2;
		OFF -= OFF2;
		SENS -= SENS2;
	}

	temperature_celsius = celsius;
	pressure_octapascal = (int32_t) (((((D1 * SENS) >> 21) - OFF) >> 12)); // gives Pascal / 8
}

void MS5611::update(void) {
	uint32_t tmp;
	if (measure_temperature) {
		tmp = read_24_bits();
		if (tmp)
			ADC_temperature_reading = tmp;
		else {
			++errorcount;
			start_temperature_conversion();
			measure_temperature = true;
			return;
		}
		start_pressure_conversion();
		measure_temperature = false;
	} else {
		tmp = read_24_bits();
		if (tmp)
			ADC_pressure_reading = tmp;
		else {
			++errorcount;
			start_pressure_conversion();
			measure_temperature = false;
			return;
		}
		start_temperature_conversion();
		measure_temperature = true;
		calibrate(ADC_pressure_reading, ADC_temperature_reading);
	}
}

inline uint16_t MS5611::read_coef(uint8_t coef_num) {
	static const uint8_t RX_BUFLEN = 2;
	uint8_t Buffer_Rx[RX_BUFLEN];
	uint8_t reg = CMD_PROM_RD + coef_num * 2;
	I2C_ReadRegister(MS5611_I2C, I2C_address, reg, 1, Buffer_Rx, RX_BUFLEN);
	return (Buffer_Rx[0] << 8) + Buffer_Rx[1];
}

inline uint32_t MS5611::getRawDx(uint8_t cmd) {
	uint8_t reg = CMD_ADC_CONV | cmd;
	I2C_Write(MS5611_I2C, I2C_address, &reg, 1);
	vTaskDelay(10);

	return read_24_bits();
}

void MS5611::initialize(void) {
	errorcount = 0;
	uint8_t reg = CMD_RESET;
	I2C_Write(MS5611_I2C, I2C_address, &reg, 1);
	vTaskDelay(3);

	for (uint8_t j = 0; j < 8; j++)
		PromData[j] = read_coef(j);

	uint8_t us_expectedCRC = PromData[7] & 0x000F;
	uint8_t uc_CRC = get_crc4();
	ASSERT(uc_CRC == us_expectedCRC);

	start_temperature_conversion();
	vTaskDelay(15);
	ADC_temperature_reading = getRawDx(CMD_ADC_D2 | CMD_ADC_4096);
	start_pressure_conversion();
	vTaskDelay(15);
	ADC_pressure_reading = getRawDx(CMD_ADC_D1 | CMD_ADC_4096);
	calibrate(ADC_pressure_reading, ADC_temperature_reading);
	start_temperature_conversion();
	measure_temperature = true;
	vTaskDelay(15);
}

#endif
