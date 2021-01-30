#include "FreeRTOS_wrapper.h"
#include "ascii_support.h"

extern ADC_HandleTypeDef hadc1;

#define ITM_TRACE 0

#define CONVERSION_FACTOR  (11.0f * 3.3f / 4096.0f)
float get_supply_voltage(void)
{
	uint32_t  value = 0;
	HAL_ADC_Start(&hadc1);
	while(HAL_OK != HAL_ADC_PollForConversion(&hadc1, 0))
	{
		delay(10);  //TODO: implement interrupt notification.
	}
	value = HAL_ADC_GetValue(&hadc1);
	return (float)value * CONVERSION_FACTOR;
}

#if ITM_TRACE
COMMON char buf[10];
#endif
void adc_measurement(void*)
{
	for(;;)
	{
		output_data.m.supply_voltage = get_supply_voltage();
		delay(100);

#if ITM_TRACE
		itoa((uint32_t)(output_data.m.supply_voltage*1000.0), buf);
		uint32_t idx = 0;
		while(buf[idx] != 0)
		{
			ITM_SendChar(buf[idx]);
			idx++;
		}
		ITM_SendChar('\r');
		ITM_SendChar('\n');
#endif
	}
}

RestrictedTask adc_reading(adc_measurement, "ADC_READ", 128);
