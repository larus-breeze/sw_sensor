#include "FreeRTOS_wrapper.h"

COMMON uint32_t allowed;
uint32_t disallowed;

void harakiri( void *)
{
  ++allowed;
  ++disallowed;

  harakiri(0); // recursive suicide call
}

// RestrictedTask commit_suicide( harakiri);


