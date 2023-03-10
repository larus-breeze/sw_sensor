#include "system_configuration.h"
#include "my_assert.h"
#include "NAV_tuning_parameters.h"
#include "embedded_math.h"

//! helper function, in use until manual configuration is implemented
void write_EEPROM_defaults( void)
{
  unsigned status;
  status = EEPROM_initialize();
  ASSERT( ! status);

  status = lock_EEPROM( false);
  ASSERT( status == 0);

  status = write_EEPROM_value( SENS_TILT_ROLL, 0.0f);
  ASSERT( ! status);
  status = write_EEPROM_value( SENS_TILT_NICK, 0.0f);
  ASSERT( ! status);
  status = write_EEPROM_value( SENS_TILT_YAW,  0.0f);   // looking backwards
  ASSERT( ! status);

  status = write_EEPROM_value( ANT_BASELENGTH, 1.0f);
  ASSERT( ! status);
  status = write_EEPROM_value( ANT_SLAVE_DOWN, 0.0f);
  ASSERT( ! status);
  status = write_EEPROM_value( ANT_SLAVE_RIGHT, 0.0f);
  ASSERT( ! status);

  status = write_EEPROM_value( GNSS_CONFIGURATION, (float)GNSS_M9N);
  ASSERT( ! status);

  status = write_EEPROM_value( PITOT_OFFSET, 0.0f);
  ASSERT( ! status);
  status = write_EEPROM_value( PITOT_SPAN, 1.0f);
  ASSERT( ! status);

  status = write_EEPROM_value( QNH_OFFSET, 0.0f);
  ASSERT( ! status);

  write_EEPROM_value( MAG_X_OFF, 0.0f);
  write_EEPROM_value( MAG_X_SCALE, 1.0f);
  write_EEPROM_value( MAG_Y_OFF, 0.0f);
  write_EEPROM_value( MAG_Y_SCALE, 1.0f);
  write_EEPROM_value( MAG_Z_OFF, 0.0f);
  write_EEPROM_value( MAG_Z_SCALE, 1.0f);
  write_EEPROM_value( MAG_STD_DEVIATION, 0.009999f);

  // time constants
  status = write_EEPROM_value( VARIO_TC, DEFAULT_VARIO_TC);
  ASSERT( ! status);
  status = write_EEPROM_value( VARIO_INT_TC, DEFAULT_AVG_VARIO_TC);
  ASSERT( ! status);
  status = write_EEPROM_value( WIND_TC, DEFAULT_WIND_TC);
  ASSERT( ! status);
  status = write_EEPROM_value( MEAN_WIND_TC, DEFAULT_WIND_AVG_TC);
  ASSERT( ! status);
  status = write_EEPROM_value( VETF, DEFAULT_VETF);
  ASSERT( ! status);

  status = write_EEPROM_value( DECLINATION, 0.0f * M_PI_F / 180.0);
  ASSERT( ! status);
  status = write_EEPROM_value( INCLINATION, +65.5f * M_PI_F / 180.0);
  ASSERT( ! status);

  lock_EEPROM( true);
}
