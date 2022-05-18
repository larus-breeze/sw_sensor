#include "system_configuration.h"
#include "main.h"
#include "FreeRTOS_wrapper.h"

static COMMON WWDG_HandleTypeDef WwdgHandle;

void heartbeat (void)
{
  bool set = GPIO_PIN_SET
      == HAL_GPIO_ReadPin ( LED_STATUS3_GPIO_Port, LED_STATUS3_Pin);
  HAL_GPIO_WritePin (LED_STATUS3_GPIO_Port, LED_STATUS3_Pin,
		     set ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

#if WATCHDOG_STATISTICS
COMMON volatile uint32_t watchdog_min=0xffffffff;
COMMON volatile uint32_t watchdog_max=0;
#endif

void blink (void*)
{
#if ACTIVATE_WATCHDOG
  acquire_privileges();

  __HAL_RCC_WWDG_CLK_ENABLE();
  WwdgHandle.Instance = WWDG;
  WwdgHandle.Init.Prescaler = WWDG_PRESCALER_8;
  WwdgHandle.Init.Window = 0x60;
  WwdgHandle.Init.Counter = 127;
  WwdgHandle.Init.EWIMode=WWDG_EWI_ENABLE;

  NVIC_SetPriority ((IRQn_Type) WWDG_IRQn, STANDARD_ISR_PRIORITY);
  NVIC_EnableIRQ ((IRQn_Type) WWDG_IRQn);

  if (HAL_WWDG_Init (&WwdgHandle) != HAL_OK)
    Error_Handler ();

  drop_privileges();

#endif // ACTIVATE_WATCHDOG


  uint8_t rythm = 0;
  for (synchronous_timer t (40); true;)
    {
      t.sync ();
      if( (++rythm & 0x0f) ==0)
	  HAL_GPIO_TogglePin (LED_STATUS3_GPIO_Port, LED_STATUS3_Pin);

#if ACTIVATE_WATCHDOG
#if WATCHDOG_STATISTICS
      uint32_t watchdog_actual = WwdgHandle.Instance->CR & 0x7f;
      if( watchdog_actual > watchdog_max)
	watchdog_max = watchdog_actual;
      if( watchdog_actual < watchdog_min)
	watchdog_min = watchdog_actual;
#endif
      if (HAL_WWDG_Refresh (&WwdgHandle) != HAL_OK)
	  Error_Handler ();
#endif
    }
}

RestrictedTask watchdog_handler (blink, "WATCHDOG", configMINIMAL_STACK_SIZE, 0, WATCHDOG_TASK_PRIORITY);

extern "C" void WWDG_IRQHandler(void)
{
  asm("bkpt 0");
}