#ifndef INC_PERSISTENT_DATA_H_
#define INC_PERSISTENT_DATA_H_

#include "stdint.h"

#define M_PI_F 3.14159265358979323846f // todo find correct placing

#ifdef __cplusplus

typedef union
  {
    uint16_t u16;
    int16_t  i16;
  } EEPROM_data_t;

class persistent_data_t
{
public:
  enum { MNEMONIC_LENGTH=16};
  uint16_t id;
  char mnemonic[MNEMONIC_LENGTH];
  EEPROM_data_t value;
};

enum EEPROM_PARAMETER_ID
{
  BOARD_ID = 0,

  SENS_TILT_ROLL = 1,
  SENS_TILT_NICK,
  SENS_TILT_YAW,

  PITOT_OFFSET=4,
  PITOT_SPAN,
  QNH_OFFSET,

  MAG_X_OFF=10,
  MAG_X_SCALE,
  MAG_Y_OFF,
  MAG_Y_SCALE,
  MAG_Z_OFF,
  MAG_Z_SCALE,
  MAG_STD_DEVIATION,

  DECLINATION=20,
  INCLINATION,

  VARIO_TC=30,
  VARIO_INT_TC,
  WIND_TC,
  MEAN_WIND_TC,

  ANT_BASELENGTH=40,
  ANT_SLAVE_DOWN,
  ANT_SLAVE_RIGHT,

  EEPROM_PARAMETER_ID_END // 1 behind last parameter ID
};

// standard function to read configratin data rem EEPROM
float configuration( EEPROM_PARAMETER_ID id);
bool write_EEPROM_value( EEPROM_PARAMETER_ID id, float value);
bool read_EEPROM_value( EEPROM_PARAMETER_ID id, float &value);
bool lock_EEPROM( bool lockit);
bool EEPROM_initialize( void);

#endif ///#ifdef __cplusplus

#endif /* INC_PERSISTENT_DATA_H_ */
