// Host stub: the FreeRTOS wrapper API used by uSD_handler.cpp. Simulated
// time advances in delay() and notify_take() (see logging_resume_test.cpp).
#pragma once
#include <stdint.h>
#include "system_configuration.h"
#define INFINITE_WAIT 0xffffffffu
#define configMAX_PRIORITIES 7
#define configMINIMAL_STACK_SIZE 128
#define portPRIVILEGE_BIT 0x80000000u
#define portMPU_REGION_READ_WRITE 0
typedef void * TaskHandle_t;
typedef struct { void *base; uint32_t length; uint32_t parameters; } MemoryRegion_t;
typedef struct
{
  void (*code)(void *); const char *name; uint32_t stack_depth; void *parameters;
  uint32_t priority; uint32_t *stack_buffer; MemoryRegion_t regions[3];
} TaskParameters_t;
void delay (unsigned ms);
uint32_t notify_take (bool clear);
inline void suspend (void) {}
inline void acquire_privileges (void) {}
inline void drop_privileges (void) {}
inline void vTaskSuspend (TaskHandle_t) {}
class Semaphore { public: void signal (void) {} };
class RestrictedTask
{
public:
  RestrictedTask (const TaskParameters_t &) {}
  RestrictedTask (void (*)(void *), const char *, uint32_t, void *, uint32_t) {}
  void set_priority (uint32_t) {}
  void notify_give (void) {}
  void notify_give_from_ISR (void) {}
  void resume_from_ISR (void) {}
  void suspend (void) {}
  TaskHandle_t get_handle (void) { return this; }
};
