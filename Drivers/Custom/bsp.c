#include "bsp_driver_sd.h"


uint8_t BSP_SD_IsDetected(void)
{
  __IO uint8_t status = SD_PRESENT;

   /* Check SD card detect pin */
   if(HAL_GPIO_ReadPin(SD_DETECT_GPIO_PORT, SD_DETECT_PIN) == GPIO_PIN_RESET)
   {
	   status = SD_NOT_PRESENT;
   }
    /* USER CODE BEGIN 1 */
    /* user code can be inserted here */
    /* USER CODE END 1 */
    return status;
}
