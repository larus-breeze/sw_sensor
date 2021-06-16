#include "system_configuration.h"
#include "main.h"
#include "FreeRTOS_wrapper.h"
#include "eeprom.h"

#if TEST_EEPROM

static void test( void *)
{
  volatile uint16_t status;
  volatile uint16_t data;

  delay(10);

  HAL_FLASH_Unlock();
  ASSERT(status == HAL_OK);

  status = EE_Init();
  ASSERT(status == HAL_OK);

  status = EE_WriteVariable( 0xf000, 0x18a5);
  ASSERT(status == HAL_OK);
  status = EE_WriteVariable( 0xf000, 0x5a81);
  ASSERT(status == HAL_OK);
  status = EE_WriteVariable( 0x001, 0x0000);
  ASSERT(status == HAL_OK);
  status = EE_ReadVariable( 1, (uint16_t*)&data);
  ASSERT(status == HAL_OK);
  suspend();
}

static ROM TaskParameters_t p =
  {
      test,
      "EEPROM",
      128,
      0,
      STANDARD_TASK_PRIORITY,
      0,
    {
      { COMMON_BLOCK, COMMON_SIZE, portMPU_REGION_READ_WRITE },
      { (void *)0x80f8000, 0x8000, portMPU_REGION_READ_WRITE },
      { 0, 0, 0 }
    }
  };

COMMON RestrictedTask eeprom_test_task (p);

#endif
