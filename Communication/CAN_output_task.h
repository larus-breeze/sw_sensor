/*
 * CAN_output_task.h
 *
 *  Created on: Sep 5, 2022
 *      Author: schaefer
 */

#ifndef CAN_OUTPUT_TASK_H_
#define CAN_OUTPUT_TASK_H_

#include "FreeRTOS_wrapper.h"

extern RestrictedTask CAN_task;

inline void trigger_CAN(void)
{
  CAN_task.notify_give();
}

#endif /* CAN_OUTPUT_TASK_H_ */
