/** ***********************************************************************
 * @file		CAN_output.h
 * @brief		format internal data and send to CAN
 * @author		Dr. Klaus Schaefer
 **************************************************************************/
#ifndef SRC_CAN_OUTPUT_H_
#define SRC_CAN_OUTPUT_H_

extern Task CAN_task;

void trigger_CAN(void)
{
  CAN_task.notify_give();
}

#endif /* SRC_CAN_OUTPUT_H_ */
