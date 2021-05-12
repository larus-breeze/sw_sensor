#include "system_configuration.h"

//! helper function, in use until manual configuration is implemented
void write_EEPROM_defaults( void)
{
  lock_EEPROM( false);

  // sensor orientation
  write_EEPROM_value( SENS_TILT_ROLL, 0.0f); // todo presently D-KCOM special data
  write_EEPROM_value( SENS_TILT_NICK, -0.13f);
  write_EEPROM_value( SENS_TILT_YAW, -3.14159265f);

  write_EEPROM_value( ANT_BASELENGTH, 2.03f);
  write_EEPROM_value( ANT_SLAVE_DOWN, 0.263f);
  write_EEPROM_value( ANT_SLAVE_RIGHT, -0.06f);

  // time constants
  write_EEPROM_value( VARIO_TC, VARIO_F_BY_FS);
  write_EEPROM_value( VARIO_INT_TC, AVG_VARIO_F_BY_FS);

  write_EEPROM_value( WIND_TC, WIND_SHORTTERM_F_BY_FS);
  write_EEPROM_value( MEAN_WIND_TC, WIND_AVG_F_BY_FS);

  write_EEPROM_value( PITOT_OFFSET, -7.5f);
  write_EEPROM_value( PITOT_SPAN, 1.031f);

  write_EEPROM_value( QNH_OFFSET, 0.0f);

  write_EEPROM_value( DECLINATION, +3.0f * M_PI_F / 180.0);
  write_EEPROM_value( INCLINATION, +65.5f * M_PI_F / 180.0);

  lock_EEPROM( true);
}
