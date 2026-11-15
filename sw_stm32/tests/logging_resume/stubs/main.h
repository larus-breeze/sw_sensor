// Host stub: the HAL/CMSIS names uSD_handler.cpp uses.
#pragma once
#include <stdint.h>
typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1 } GPIO_PinState;
typedef struct { int dummy; } GPIO_TypeDef, SD_HandleTypeDef, DMA_HandleTypeDef;
extern GPIO_TypeDef host_gpio;
#define LED_STATUS1_GPIO_Port (&host_gpio)
#define LED_STATUS2_Pin 1
#define LED_ERROR_GPIO_Port (&host_gpio)
#define LED_ERROR_Pin 2
inline void HAL_GPIO_WritePin (GPIO_TypeDef *, uint16_t, GPIO_PinState) {}
HAL_StatusTypeDef HAL_SD_Init (SD_HandleTypeDef *);
HAL_StatusTypeDef HAL_SD_DeInit (SD_HandleTypeDef *);
bool BSP_PlatformIsDetected (void);
#define RCC_FLAG_PORRST 1
#define RCC_FLAG_BORRST 2
#define __HAL_RCC_GET_FLAG(f) (0)
#define __HAL_RCC_CLEAR_RESET_FLAGS() do {} while (0)
// flash: the staging area is read through fixed addresses (see host_flash.cpp)
#define FLASH_SECTOR_7 7
#define FLASH_SECTOR_8 8
#define FLASH_SECTOR_9 9
#define FLASH_FLAG_EOP 0
#define FLASH_FLAG_OPERR 0
#define FLASH_FLAG_WRPERR 0
#define FLASH_FLAG_PGAERR 0
#define FLASH_FLAG_PGPERR 0
#define FLASH_FLAG_PGSERR 0
#define __HAL_FLASH_CLEAR_FLAG(f) do {} while (0)
#define FLASH_TYPEERASE_SECTORS 0
#define VOLTAGE_RANGE_3 2
typedef struct { uint32_t TypeErase, Banks, Sector, NbSectors, VoltageRange; } FLASH_EraseInitTypeDef;
inline HAL_StatusTypeDef HAL_FLASH_Unlock (void) { return HAL_ERROR; }
inline HAL_StatusTypeDef HAL_FLASH_Lock (void) { return HAL_OK; }
inline HAL_StatusTypeDef HAL_FLASHEx_Erase (FLASH_EraseInitTypeDef *, uint32_t *) { return HAL_ERROR; }
#define __ALIGNED(x) __attribute__((aligned(x)))
inline uint32_t host_get_IPSR (void) { return 0; } // task context
