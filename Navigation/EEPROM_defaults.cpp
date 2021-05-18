#include "system_configuration.h"
#include "compass_calibration.h"

//! helper function, in use until manual configuration is implemented
void write_EEPROM_defaults( void)
{
  unsigned status;
  status = EEPROM_initialize();
  ASSERT( ! status);

  status = lock_EEPROM( false);
  ASSERT( status == HAL_OK);

#if DKCOM == 1 // *******************************************************************

  // sensor orientation
  write_EEPROM_value( SENS_TILT_ROLL, 0.0f); // todo presently D-KCOM special data
  write_EEPROM_value( SENS_TILT_NICK, -0.13f);
  write_EEPROM_value( SENS_TILT_YAW, -3.14159265f);

  write_EEPROM_value( ANT_BASELENGTH, 2.03f);
  write_EEPROM_value( ANT_SLAVE_DOWN, 0.263f);
  write_EEPROM_value( ANT_SLAVE_RIGHT, -0.06f);
#else
  // sensor orientation
  write_EEPROM_value( SENS_TILT_ROLL, 3.14159265f); 	// sensor orientation USB -> front
  write_EEPROM_value( SENS_TILT_NICK, 0.0f);		// component side top
  write_EEPROM_value( SENS_TILT_YAW,  0.0f);
#endif
  // time constants
  write_EEPROM_value( VARIO_TC, VARIO_F_BY_FS);
  write_EEPROM_value( VARIO_INT_TC, AVG_VARIO_F_BY_FS);

  write_EEPROM_value( WIND_TC, WIND_SHORTTERM_F_BY_FS);
  write_EEPROM_value( MEAN_WIND_TC, WIND_AVG_F_BY_FS);

  write_EEPROM_value( PITOT_OFFSET, -7.5f);
  write_EEPROM_value( PITOT_SPAN, 1.031f);

  write_EEPROM_value( QNH_OFFSET, 0.0f);
#if 1
  write_EEPROM_value( MAG_X_OFF, 0.0f);
  write_EEPROM_value( MAG_X_SCALE, 1.0f);
  write_EEPROM_value( MAG_Y_OFF, 0.0f);
  write_EEPROM_value( MAG_Y_SCALE, 1.0f);
  write_EEPROM_value( MAG_Z_OFF, 0.0f);
  write_EEPROM_value( MAG_Z_SCALE, 1.0f);
  write_EEPROM_value( MAG_VARIANCE, 0.9999e-5f);
#else // used to test compass calibration + EEPROM writing
  compass_calibration_t calibration;
  linear_least_square_fit<float> mag_calibrator[3];

  mag_calibrator[0].add_value(0.0f, 0.33f);
  mag_calibrator[0].add_value(1.0f, 1.0f + 0.33f);
  mag_calibrator[0].add_value(2.0f, 2.0f * (1.0f + 0.33f) + 0.1f);

  mag_calibrator[1].add_value(0.0f, 0.0f);
  mag_calibrator[1].add_value(1.0f, 1.0f);
  mag_calibrator[1].add_value(2.0 * 1.0f, 2.0f * 1.0f + 0.1f);

  mag_calibrator[2].add_value(0.0f, -3.0f);
  mag_calibrator[2].add_value(1.0f, 1.0f-3.0f);
  mag_calibrator[2].add_value(2.0f , 2.0f -3.0f + 0.1f);

  calibration.set_calibration(mag_calibrator, 'X', false);
  calibration.write_into_EEPROM();
#endif
  write_EEPROM_value( QNH_OFFSET, 0.0f);

  write_EEPROM_value( DECLINATION, +3.0f * M_PI_F / 180.0);
  write_EEPROM_value( INCLINATION, +65.5f * M_PI_F / 180.0);



  lock_EEPROM( true);
}
