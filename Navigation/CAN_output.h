/*
 * CAN_output.h
 *
 *  Created on: Dec 29, 2020
 *      Author: schaefer
 */

#ifndef SRC_CAN_OUTPUT_H_
#define SRC_CAN_OUTPUT_H_

extern Task CAN_task;

void trigger_CAN(void)
{
  CAN_task.notify_give();
}

#endif /* SRC_CAN_OUTPUT_H_ */
