#include "system_configuration.h"
#include "my_assert.h"
#include "eeprom.h"
#include "embedded_memory.h"
#include "persistent_data.h"

ROM persistent_data_t persistent_data[]=
    {
	{BOARD_ID, 	"Board_ID", 0},	 	//! Board ID Hash to avoid board confusion

	{SENS_TILT_ROLL,"SensTilt_Roll", 0}, 	//! IMU Sensor tilt angle signed / (pi / 32768) front right down frame
	{SENS_TILT_NICK,"SensTilt_Nick", 0}, 	//! IMU Sensor tilt angle signed / (pi / 32768)
	{SENS_TILT_YAW, "SensTilt_Yaw", 0},  	//! IMU Sensor tilt angle signed / (pi / 32768)

	{PITOT_OFFSET,	"Pitot_Offest", 0},	//! Pitot offset signed / ( ADC readings )
	{PITOT_SPAN, 	"Pitot_Span", 0},	//! Pitot Span signed ( scale-factor  = value / 32768 + 1.0f)
	{QNH_OFFSET, 	"QNH-delta", 0},	//! Absolute pressure sensor offset signed / Pa

	{MAG_X_OFF,	"Mag_X_Off", 0},	//! Induction sensor x offset signed / ( 10.0f / 32768 )
	{MAG_X_GAIN,	"Mag_X_Gain", 0},	//! Induction sensor x gain signed ( scale-factor = 1.0f + value / 32768 )
	{MAG_Y_OFF,	"Mag_Y_Off", 0},	//! Induction sensor x offset signed / ( 10.0f / 32768 )
	{MAG_Y_GAIN,	"Mag_Y_Gain", 0},	//! Induction sensor x gain signed ( scale-factor = 1.0f + value / 32768 )
	{MAG_Z_OFF,	"Mag_Z_Off", 0},	//! Induction sensor x offset signed / ( 10.0f / 32768 )
	{MAG_Z_GAIN,	"Mag_Z_Gain", 0},	//! Induction sensor x gain signed ( scale-factor = 1.0f + value / 32768 )
	{MAG_VARIANCE,	"Mag_Calib_Var", 0}, 	//! Magnetic calibration variance unsigned / ( 1e-5f / 65536 )

	{DECLINATION,	"Mag_Declination", 0}, 	//! Magnetic declination (east positive) signed / ( 180° / 32768)
	{INCLINATION,	"Mag_Inclination", 0}, 	//! Magnetic inclination (down positive) signed / ( 180° / 32768)

	{VARIO_TC,	"Vario_TC", 0},	 	//! Vario time constant unsigned s / ( 100.0f / 65536 )
	{VARIO_INT_TC,	"Vario_Int_TC", 0},	//! Vario integrator time constant unsigned s / ( 100.0f / 65536 )
	{WIND_TC,	"Wind_TC", 0},	 	//! Wind fast time constant unsigned s / ( 100.0f / 65536 )
	{MEAN_WIND_TC,	"Mean_Wind_TC", 0},	//! Wind slow time constant unsigned s / ( 100.0f / 65536 )

	{ANT_BASELENGTH, "ANT_BASELEN", 0},	//! Slave DGNSS antenna baselength / mm
	{ANT_SLAVE_DOWN, "ANT_SLAVE_DOWN", 0},	//! Slave DGNSS antenna lower / mm
	{ANT_SLAVE_RIGHT,"ANT_SLAVE_RIGHT", 0},	//! Slave DGNSS antenna more right /mm
    };

#define READ true
#define WRITE false

bool EEPROM_convert( EEPROM_PARAMETER_ID id, EEPROM_data_t & EEPROM_value, float & value , bool read )
{
  switch(id)
  {
    case BOARD_ID:
      if( read)
	value = (float)EEPROM_value.u16;
      else
	EEPROM_value.u16 = (uint16_t)value;
      break;
    case QNH_OFFSET:
    case PITOT_OFFSET:
      if( read)
	value = ( float)EEPROM_value.i16;
      else
	EEPROM_value.i16 = (int16_t)value;
      break;
    case PITOT_SPAN:
    case MAG_X_GAIN:
    case MAG_Y_GAIN:
    case MAG_Z_GAIN:
      if( read)
	value = ( (float)EEPROM_value.i16 / 32768.0f) + 1.0f;
      else
	EEPROM_value.i16 = (int16_t)((value - 1.0f) * 32768.0f);
      break;
    case MAG_VARIANCE:
      if( read)
	value = (float)EEPROM_value.u16 / 65536.0f * 1e-5f;
      else
	EEPROM_value.u16 = (uint16_t)(value * 1e5f * 65536.0f);
      break;
    case MAG_X_OFF:
    case MAG_Y_OFF:
    case MAG_Z_OFF:
      if( read)
	value = ( (float)EEPROM_value.i16 / 3276.8f) + 1.0f;
      else
	EEPROM_value.i16 = (uint16_t)(value - 1.0f * 3276.8f);
      break;
    case ANT_BASELENGTH: // max +/- 32.768 m
    case ANT_SLAVE_DOWN:
    case ANT_SLAVE_RIGHT:
      if( read)
	value = (float)EEPROM_value.i16 * 0.001f;
      else
	EEPROM_value.i16 = (int16_t)(value * 1000.0f);
      break;
      break;
    case DECLINATION:
    case INCLINATION:
    case SENS_TILT_ROLL:
    case SENS_TILT_NICK:
    case SENS_TILT_YAW:
      if( read)
	value = (float)EEPROM_value.i16 / 32768.0f * M_PI_F;
      else
	EEPROM_value.i16 = (int16_t)(value * 32768.0f / M_PI_F);
      break;
    case VARIO_TC:
    case VARIO_INT_TC:
    case WIND_TC:
    case MEAN_WIND_TC:
      if( read)
	value = (float)EEPROM_value.u16 / 655.36f;
      else
	EEPROM_value.u16 = (uint16_t)(value * 655.36f);
      break;
    case EEPROM_PARAMETER_ID_END:
    default:
	return true; // error
      break;
  }
  return false; // OK
}

bool lock_EEPROM( bool lockit)
{
  if( lockit)
    return HAL_FLASH_Lock();

  HAL_FLASH_Unlock();
  return (EE_Init());
}

bool write_EEPROM_value( EEPROM_PARAMETER_ID id, float value)
{
  EEPROM_data_t EEPROM_value;
  if( EEPROM_convert( id, EEPROM_value, value , WRITE))
      return true; // error

  EEPROM_data_t read_value;
  if( HAL_OK == EE_ReadVariable( id, &read_value.u16) && (read_value.u16 == EEPROM_value.u16))
    return HAL_OK;
  return EE_WriteVariable( id, EEPROM_value.u16);
}

bool read_EEPROM_value( EEPROM_PARAMETER_ID id, float &value)
{
  uint16_t data;
  if( HAL_OK != EE_ReadVariable( id, (uint16_t*)&data))
    return true;
  return ( EEPROM_convert( id, (EEPROM_data_t &)data, value , READ));
}

float configuration( EEPROM_PARAMETER_ID id)
{
  float value;
  bool result = read_EEPROM_value( id, value);
  ASSERT( result == false);
  return value;
}
