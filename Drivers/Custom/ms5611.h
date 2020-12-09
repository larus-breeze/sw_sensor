#ifndef MS5611_DRIVER_H
#define MS5611_DRIVER_H

#include <i2c.h>

class MS5611
{
public:
  inline MS5611(uint8_t i2c_address)
  {
	  I2C_address = i2c_address;
  }
  void initialize(void);
  void update( void);
  inline float get_pressure( void) const //!< getter function
  {
    return pressure_octapascal * 0.125f;
  }
  inline int32_t get_pressure_octapascal( void) const //!< getter function
  {
    return pressure_octapascal;
  }
  inline float get_temperature(void) const
  {
	  return temperature_celsius * 0.01f;
  }
  void start_pressure_conversion( void);   //TODO: shall this be private functions instead?
  void start_temperature_conversion( void);

private:
  inline uint16_t read_coef (uint8_t coef_num);
  inline uint8_t get_crc4 ();
  inline void calibrate( const uint32_t D1, const uint32_t D2);
  inline uint32_t read_24_bits();
  inline uint32_t getRawDx (uint8_t cmd);


  uint8_t I2C_address; //!< I2C address
  uint32_t ADC_temperature_reading; 	//!< 24 bits ADC value of the temperature conversion
  uint32_t ADC_pressure_reading;    	//!< 24 bits ADC value of the pressure conversion
  int32_t  pressure_octapascal; 	//!< absolute pressure in 1/8 Pascal
  int32_t  temperature_celsius;		//!< sensor temperature in 1/100 Degrees Celsius
  uint16_t PromData[8]; 		//!< coefficients table for pressure sensor PROM values
  bool measure_temperature;	//!< Measurement cycle flip-flop
  uint32_t errorcount;
};

#endif /* MS5611_01BA01_H_ */

