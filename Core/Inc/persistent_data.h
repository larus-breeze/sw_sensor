#ifndef INC_PERSISTENT_DATA_H_
#define INC_PERSISTENT_DATA_H_

#include "stdint.h"
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

typedef enum EEPROM_PARAMETER_ID
{
  BOARD_ID = 0,

  SENS_TILT_ROLL = 1,
  SENS_TILT_NICK,
  SENS_TILT_YAW,

  PITOT_OFFSET=4,
  PITOT_SPAN,
  QNH_DELTA,

  MAG_X_OFF=10,
  MAG_X_GAIN,
  MAG_Y_OFF,
  MAG_Y_GAIN,
  MAG_Z_OFF,
  MAG_Z_GAIN,
  MAG_VARIANCE,

  DEKLINATION=20,
  INKLINATION,

  VARIO_TC=30,
  VARIO_INT_TC,
  WIND_TC,
  MEAN_WIND_TC,

  ANT_DOWN=40,
  ANT_RIGHT,

  EEPROM_PARAMETER_ID_END // 1 behind last parameter ID
};

float EEPROM_to_value( EEPROM_PARAMETER_ID id, EEPROM_data_t value);

#endif /* INC_PERSISTENT_DATA_H_ */
