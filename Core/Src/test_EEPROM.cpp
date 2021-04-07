#include "system_configuration.h"
#include "main.h"
#include "FreeRTOS_wrapper.h"
#include "eeprom.h"

#if TEST_EEPROM

static void test( void *)
{
  volatile uint16_t status;
  volatile uint16_t data;

  HAL_FLASH_Unlock();
  status = EE_Init();
  ASSERT(status == HAL_OK);
  status = EE_WriteVariable( 1, 0xa512);
  ASSERT(status == HAL_OK);
  status = EE_ReadVariable( 1, (uint16_t*)&data);
  ASSERT(status == HAL_OK);
  suspend();
}

static TaskParameters_t p =
  {
      test,
      "EEPROM",
      128,
      0,
      STANDARD_TASK_PRIORITY,
      0,
    {
      { COMMON_BLOCK, COMMON_SIZE, portMPU_REGION_READ_WRITE },
      { (void *)0x80f8000, 0x10000, portMPU_REGION_READ_WRITE },
      { 0, 0, 0 }
    }
  };

COMMON RestrictedTask eeprom_test_task (p);

#endif
