
#include "system_configuration.h"
#include "main.h"
#include "FreeRTOS_wrapper.h"
#include "embedded_math.h"
#include "magnetic_matrix_calculator.h"
#include "magnetic_induction_report.h"

COMMON compass_calibrator_3D_t compass_calibrator_3D;
COMMON Semaphore calculation_trigger;

void trigger_compass_calibrator_3D_calculation(void)
{
  calculation_trigger.signal();
}

static void magnetic_calculator_runnable ( void *)
{
  while( true)
    {
      calculation_trigger.wait();
      compass_calibrator_3D.calculate();
      report_magnetic_calibration_has_changed( 0, '3');
    }
}

#define STACKSIZE 1024
static uint32_t __ALIGNED(STACKSIZE*4) stack_buffer[STACKSIZE];

static TaskParameters_t p =
{
  magnetic_calculator_runnable,
  "mag_calc",
  STACKSIZE,
  0,
  MAG_CALCULATOR_PRIORITY,
  stack_buffer,
    {
      { COMMON_BLOCK, COMMON_SIZE, portMPU_REGION_READ_WRITE },
      { 0, 0, 0},
      { 0, 0, 0}
    }
};

static RestrictedTask magnetic_calculator_task (p);

