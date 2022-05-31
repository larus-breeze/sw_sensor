#include "system_configuration.h"
#include "compass_calibration.h"

//! helper function, in use until manual configuration is implemented
void write_EEPROM_defaults( void)
{
  unsigned status;
  status = EEPROM_initialize();
  ASSERT( ! status);

//  status = lock_EEPROM( false);
//  ASSERT( status == 0); // HAL_OK

#if DKCOM == 1 // *******************************************************************

  // sensor orientation
  status = write_EEPROM_value( SENS_TILT_ROLL, 0.0f); // todo presently D-KCOM special data
  ASSERT( ! status);
  status = write_EEPROM_value( SENS_TILT_NICK, -0.12f);
  ASSERT( ! status);
  status = write_EEPROM_value( SENS_TILT_YAW, -3.14159265f);
  ASSERT( ! status);

  status = write_EEPROM_value( GNSS_CONFIGURATION, (float)GNSS_F9P_F9H);
  ASSERT( ! status);
  status = write_EEPROM_value( ANT_BASELENGTH, 2.03f);
  ASSERT( ! status);
  status = write_EEPROM_value( ANT_SLAVE_DOWN, 0.263f);
  ASSERT( ! status);
  status = write_EEPROM_value( ANT_SLAVE_RIGHT, -0.06f);
  ASSERT( ! status);
// fine-tuned magnetic defaults D-KCOM May 2021
  status = write_EEPROM_value( MAG_X_OFF, 3.135242e-1f);
  ASSERT( ! status);
  status = write_EEPROM_value( MAG_X_SCALE, 1.0f / 1.12360e0f);
  ASSERT( ! status);
  status = write_EEPROM_value( MAG_Y_OFF, -4.493407e-3f);
  ASSERT( ! status);
  status = write_EEPROM_value( MAG_Y_SCALE, 1.0f / 9.312426e-1f);
  ASSERT( ! status);
  status = write_EEPROM_value( MAG_Z_OFF, -2.968704e0f);
  ASSERT( ! status);
  status = write_EEPROM_value( MAG_Z_SCALE, 1.0f / 8.819322e-1f);
  ASSERT( ! status);
  status = write_EEPROM_value( MAG_STD_DEVIATION, 3e-4f);
  ASSERT( ! status);

  status = write_EEPROM_value( PITOT_OFFSET, -7.5f);
  ASSERT( ! status);
  status = write_EEPROM_value( PITOT_SPAN, 1.031f);
  ASSERT( ! status);

#else
#if 0
  // sensor orientation SteFly
  status = write_EEPROM_value( SENS_TILT_ROLL, -3.14159265f);
  ASSERT( ! status);
  status = write_EEPROM_value( SENS_TILT_NICK, 0.16f);
  ASSERT( ! status);
  status = write_EEPROM_value( SENS_TILT_YAW,  -3.14159265f);   // looking backwards
  ASSERT( ! status);
  status = write_EEPROM_value( ANT_BASELENGTH, 2.8f);
  ASSERT( ! status);
  status = write_EEPROM_value( ANT_SLAVE_DOWN, 0.1f);
  ASSERT( ! status);
  status = write_EEPROM_value( ANT_SLAVE_RIGHT, 0.0f);
  ASSERT( ! status);
#else
  status = write_EEPROM_value( SENS_TILT_ROLL, -3.14159265f);
  ASSERT( ! status);
  status = write_EEPROM_value( SENS_TILT_NICK, 0.0f);
  ASSERT( ! status);
  status = write_EEPROM_value( SENS_TILT_YAW,  -3.14159265f);   // looking backwards
  ASSERT( ! status);

  status = write_EEPROM_value( ANT_BASELENGTH, 1.0f);
  ASSERT( ! status);
  status = write_EEPROM_value( ANT_SLAVE_DOWN, 0.0f);
  ASSERT( ! status);
  status = write_EEPROM_value( ANT_SLAVE_RIGHT, 0.0f);
  ASSERT( ! status);
#endif

  status = write_EEPROM_value( GNSS_CONFIGURATION, (float)GNSS_F9P_F9P);
//  status = write_EEPROM_value( GNSS_CONFIGURATION, (float)GNSS_M9N);
  ASSERT( ! status);

  status = write_EEPROM_value( PITOT_OFFSET, 0.0f);
  ASSERT( ! status);
  status = write_EEPROM_value( PITOT_SPAN, 1.0f);
  ASSERT( ! status);

  status = write_EEPROM_value( QNH_OFFSET, 0.0f);
  ASSERT( ! status);

  // simple magnetic defaults

  write_EEPROM_value( MAG_X_OFF, 0.0f);
  write_EEPROM_value( MAG_X_SCALE, 1.0f);
  write_EEPROM_value( MAG_Y_OFF, 0.0f);
  write_EEPROM_value( MAG_Y_SCALE, 1.0f);
  write_EEPROM_value( MAG_Z_OFF, 0.0f);
  write_EEPROM_value( MAG_Z_SCALE, 1.0f);
  write_EEPROM_value( MAG_STD_DEVIATION, 0.009999f);

#endif
  // time constants
  status = write_EEPROM_value( VARIO_TC, VARIO_F_BY_FS);
  ASSERT( ! status);
  status = write_EEPROM_value( VARIO_INT_TC, AVG_VARIO_F_BY_FS);
  ASSERT( ! status);

  status = write_EEPROM_value( WIND_TC, WIND_SHORTTERM_F_BY_FS);
  ASSERT( ! status);
  status = write_EEPROM_value( MEAN_WIND_TC, WIND_AVG_F_BY_FS);
  ASSERT( ! status);

#if 0 // used to test compass calibration + EEPROM writing
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

  status = write_EEPROM_value( DECLINATION, +3.0f * M_PI_F / 180.0);
  ASSERT( ! status);
  status = write_EEPROM_value( INCLINATION, +65.5f * M_PI_F / 180.0);
  ASSERT( ! status);

  lock_EEPROM( true);
}
