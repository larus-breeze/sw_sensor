#include "system_configuration.h"

#if RUN_CAN_OUTPUT == 1

#include <FreeRTOS_wrapper.h>
#include "navigator.h"
#include "flight_observer.h"
#include "serial_io.h"
#include "NMEA_format.h"
#include "common.h"
#include "candriver.h"

enum CAN_ID_SENSOR
{
  c_CAN_Id_EulerAngles	= 0x101,    //!< int16_t roll nick yaw / 1/1000 rad
  c_CAN_Id_Airspeed     = 0x102,    //!< uint16_t TAS, IAS / km/h
  c_CAN_Id_Vario		= 0x103,    //!< int16_t vario, vario-integrator / mm/s
  c_CAN_Id_GPS_Date_Time= 0x104,    //!< uint8_t year-2000, month, day, hour, mins, secs
  c_CAN_Id_GPS_LatLon	= 0x105,    //!< int32_t lat, lon / 10^-7 degrees
  c_CAN_Id_GPS_Alt		= 0x106,	//!< int64_t MSL altitude / mm
  c_CAN_Id_GPS_Trk_Spd	= 0x107,    //!< int16_t ground vector / 1/1000 rad, uint16_t groundspeed / km/h
  c_CAN_Id_Wind			= 0x108,    //!< int16_t 1/1000 rad , uint16_t km/h
  c_CAN_Id_Atmosphere	= 0x109,	//!< uint16_t pressure / Pa uint16_t density / g/m^3
  c_CAN_Id_GPS_Sats		= 0x10a,    //!< uin8_t No of Sats, Fix-Type NO=0 2D=1 3D=2 RTK=3
  c_CAN_Id_Acceleration = 0x10b,	//!< int16_t G-force mm / s^2 // fixme: changed to int16_t  HMR 17.11.20
  c_CAN_Id_TurnCoord	= 0x10c,		//!< slip angle int16_t 1/1000 rad, turn rate int16_t 1/1000 rad/s
  c_CAN_Id_SystemState	= 0x10d		//!< slip angle int16_t 1/1000 rad, turn rate int16_t 1/1000 rad/s
};

static inline float sqr(float x)
{
  return x*x;
}

void CAN_output ( const output_data_t &x)
{
  CANpacket p;

  p.id=c_CAN_Id_EulerAngles;		// 0x101
  p.dlc=6;
  p.data_sh[0] = x.euler.r * 1000.0f; 	// unit = 1/1000 RAD
  p.data_sh[1] = x.euler.n * 1000.0f;
  p.data_sh[2] = x.euler.y * 1000.0f ;
  CAN_driver.send(p, 1);

  p.id=c_CAN_Id_Airspeed;		// 0x102
  p.dlc=4;
  p.data_sh[0] = x.TAS * 3.6f; 		// m/s -> km/h
  p.data_sh[1] = x.IAS * 3.6f; 		// m/s -> km/h
  CAN_driver.send(p, 1);

  p.id=c_CAN_Id_Vario;			// 0x103
  p.dlc=4;
  p.data_sh[0] = x.vario * 1000.0f; 		// mm/s
  p.data_sh[1] = x.integrator_vario * 1000.0f; 	// mm/s
  CAN_driver.send(p, 1);

  p.id=c_CAN_Id_GPS_Date_Time;		// 0x104
  p.dlc=6;
  p.data_b[0] = x.c.year;
  p.data_b[1] = x.c.month;
  p.data_b[2] = x.c.day;
  p.data_b[3] = x.c.hour;
  p.data_b[4] = x.c.minute;
  p.data_b[5] = x.c.second;
  CAN_driver.send(p, 1);

  p.id=c_CAN_Id_GPS_LatLon;		// 0x105
  p.dlc=8;
  p.data_sw[0] = x.c.latitude * 1e7d;
  p.data_sw[1] = x.c.longitude * 1e7d;  //
  CAN_driver.send(p, 1);

  p.id=c_CAN_Id_GPS_Alt;		// 0x106
  p.dlc=8;
  p.data_l = x.c.position.e[DOWN] * -1e3f;// in mm
  CAN_driver.send(p, 1);

  p.id=c_CAN_Id_GPS_Trk_Spd;		// 0x107
  p.dlc=4;
  p.data_sh[0] = x.c.heading_motion * 17.4533f; // 1/1000 rad
  p.data_h[1] = x.c.speed_motion * 3.6f;
  CAN_driver.send(p, 1);

  p.id=c_CAN_Id_Wind;			// 0x108
  p.dlc =4;
  float wind_direction = my_atan2f( - x.wind.e[EAST], - x.wind.e[NORTH]);
  if( wind_direction < 0.0f)
    wind_direction += 6.2832f;
  p.data_sh[0] = wind_direction * 1000.0f; // 1/1000 rad
  p.data_h[1] = VSQRTF( sqr(x.wind.e[EAST])+ sqr(x.wind.e[NORTH])) * 3.6f;
  CAN_driver.send(p, 1);

  p.id=c_CAN_Id_Atmosphere;		// 0x109
  p.dlc=8;
  p.data_w[0] = x.m.static_pressure;
  p.data_w[1] = 1225; // todo: fixme: dummy
  CAN_driver.send(p, 1);

  p.id=c_CAN_Id_GPS_Sats;		// 0x10a
  p.dlc=2;
  p.data_b[0] = 30; // todo: fixme: dummy
  p.data_b[1] = 2; // todo: fixme: dummy
  CAN_driver.send(p, 1);

  p.id=c_CAN_Id_Acceleration;		// 0x10b
  p.dlc=7;
  p.data_sh[0] = x.m.acc.e[DOWN] * -1000.0f; // mm/s^2
  p.data_sh[1] = x.effective_vertical_acceleration * -1000.0f; // mm/s^2
  p.data_sh[2] = x.vario_uncompensated * 1000.0f; // mm/s
  p.data_sb[6] = x.circle_mode;
  CAN_driver.send(p, 1);

  p.id=c_CAN_Id_TurnCoord;				// 0x10c
  p.dlc=4;
  p.data_sh[0] = x.slip_angle * 1000.0f;	// mm/s^2
  p.data_sh[1] = x.turn_rate * 1000.0f; 	// mm/s^2
  if( CAN_driver.send(p, 1)) // check CAN for timeout this time
	system_state |= CAN_OUTPUT_ACTIVE;

  p.id=c_CAN_Id_SystemState;				// 0x10d
  p.dlc=4;
  p.data_w[0] = system_state;
  CAN_driver.send(p, 1);
}

void CAN_task_runnable( void *)
{
  while( true)
    {
      notify_take();
      CAN_output( output_data);
    }
}

COMMON RestrictedTask CAN_task( CAN_task_runnable, "CAN", 256, 0, CAN_PRIORITY);

#endif
