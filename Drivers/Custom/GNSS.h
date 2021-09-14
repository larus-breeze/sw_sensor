/**
 @file GNSS.h
 @brief uBlox GNSS + D-GNSS interface
 @author: Dr. Klaus Schaefer
 */
#ifndef DRIVER_GPS_H_
#define DRIVER_GPS_H_

#include "system_configuration.h"
#include "float3vector.h"

enum { NORTH, EAST, DOWN};

extern bool GNSS_new_data_ready;
extern bool D_GNSS_new_data_ready;

typedef struct
{
  uint32_t iTOW; // time of week
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint8_t valid;	// bits MSB -> LSB mag-decl tim-res time data
  uint32_t tAcc; 	// timing accuracy
  int32_t nano; 	// time fraction, signed
  uint8_t fix_type;
  uint8_t fix_flags;
  uint8_t reserved1;
  uint8_t num_SV;
  int32_t longitude;	// 10^-7 deg
  int32_t latitude;
  int32_t height_ellip;	// WGS84 height / mm
  int32_t height; 	// MSL height / mm
  uint32_t hAcc;	// horizontal accuracy / mm
  uint32_t vAcc;	// vertical accuracy / mm
  int32_t velocity[3]; 	// NED velocity mm/s
  uint32_t gSpeed; 	// Ground speed / mm/s
  uint32_t gTrack;	// track direction / 10^-5 deg
  uint32_t sAcc; 	// speed accuracy mm/s
  uint32_t headAcc;	// heading accuracy 10^-5 deg
  uint16_t pDOP;	// 0.01 units
  uint8_t reserved[14]; // useless
} uBlox_pvt;

typedef struct
{
  uint8_t version; 	// =0x01
  uint8_t dummy;	// reserved
  uint16_t ref_ID;	// ref station ID=0..4095
  uint32_t TOW;		// time of week
  int32_t relPosN;	// rel pos N / cm
  int32_t relPosE;	// rel pos E / cm
  int32_t relPosD;	// rel pos D / cm
  int32_t relPoslength;	// rel pos length / cm
  int32_t relPosheading;// rel pos heading / 1E⁻5 degrees
  uint32_t dummy1;	// reserved
  int8_t relPosHP_N;	// high precision north component / 0.1mm
  int8_t relPosHP_E;	// high precision east  component / 0.1mm
  int8_t relPosHP_D;	// high precision down  component / 0.1mm
  int8_t relPosHP_len;	// high precision length / 0.1mm
  uint32_t accN;	// accuracy north / 0.1mm
  uint32_t accE;	// accuracy north / 0.1mm
  uint32_t accD;	// accuracy north / 0.1mm
  uint32_t acc_len;	// accuracy length / 0.1mm
  uint32_t acc_heading;	// accuracy heading / 1e-5 degrees
  uint32_t dummy2;	// reserved
  uint32_t flags;	// 0b1100110111 if optimal result
} uBlox_relpos_NED;

typedef enum { FIX_none, FIX_dead, FIX_2d, FIX_3d} FIX_TYPE;
typedef enum { GNSS_HAVE_FIX, GNSS_NO_FIX, GNSS_ERROR} GNSS_Result;

typedef struct
{
  float3vector position;  	//!< NED / meters
  float3vector velocity;  	//!< NED / m/s
  float3vector acceleration;  	//!< NED / m/s^2 (from velocity delta)
  float  heading_motion;	// degrees
  float  speed_motion;		// m/s
  float3vector relPosNED;	//
  float relPosHeading;
#if OLD_COORD_FORMAT == 0
  float speed_acc;		// speed accuracy m/s
#endif
  double latitude;		//!< degrees
  double longitude;		//!< degrees
//  uint32_t time; 		// time of day / ms
//  uint32_t date; 		// calendar date 1000*year + day of year
  uint8_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
#if OLD_COORD_FORMAT == 0
  int32_t nano;		// nanoseconds from time stamp
#endif
  int16_t geo_sep_dm;		// (WGS ellipsoid height - elevation MSL) in 0.1m units
} coordinates_t;


class GNSS_type
{
public:
  GNSS_type (coordinates_t & coo);
  GNSS_Result update( const uint8_t * data);
  GNSS_Result update_delta( const uint8_t * data);
  GNSS_Result update_combined( uint8_t * data);

  void reset_reference( void)
  {
    fix_type = FIX_none;
    latitude_reference = 0; // will be updated on next fix
  }
  int64_t FAT_time;
  FIX_TYPE fix_type;
  uint8_t num_SV;

  coordinates_t &coordinates;

private:
  inline bool checkSumCheck ( const uint8_t *buffer, uint8_t length)
  {
    if( (buffer[2] != length) && (buffer[3] !=0))
        return false;

    //Checksum A and B
    uint8_t CK_A = 0, CK_B = 0;
    for (int i = 0; i < (length + 4); i++)
  	{
  	  CK_A = CK_A + buffer[i];
  	  CK_B = CK_B + CK_A;
  	}
   return ((CK_A == buffer[length + 4]) && (CK_B == buffer[length + 5]));
  }

  int32_t latitude_reference;
  int32_t longitude_reference;
  float latitude_scale;
  unsigned old_timestamp_ms;
};

extern GNSS_type GNSS;

#endif /* DRIVER_GPS_H_ */
