/**
 @file pitot_sensor.cpp
 @brief HCLA pressure sensor driver
 @author: Klaus Schaefer
 */
#include "system_configuration.h"
#include "main.h"
#include "FreeRTOS_wrapper.h"
#include "i2c.h"
#include "common.h"

#if RUN_PITOT_MODULE

#define I2C_ADDRESS (0x28<<1) // 7 bits left-adjusted
/*
 * Bereich: 80% von 16384 counts auf 1PSI verteilt
 * ergibt 6895 Pa / 13107 counts = 0.5261 Pa / count
 */

#define SPAN 0.5261f
#define OFFSET 1638 // exakt 0.1 * 16384

static void runnable( void *)
{
  uint8_t data[4];

  I2C_Init( &hi2c1);

  drop_privileges();

  for( synchronous_timer t(10); true; t.sync())
    {
      I2C_Read( &hi2c1, I2C_ADDRESS, data, 2);
      ASSERT(( data[0] & 0xC0)==0); 			// no error flags !
      uint16_t raw_data = (data[0] << 8) | data[1];
      output_data.m.pitot_pressure = (float)( raw_data - OFFSET) * SPAN;
    }
}

RestrictedTask pitot_reading ( runnable, "PITOT", 256, 0, PITOT_PRIORITY + portPRIVILEGE_BIT);

#endif

