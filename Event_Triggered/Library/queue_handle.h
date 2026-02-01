#ifndef _QUEUE_HANDLE_H
#define _QUEUE_HANDLE_H

#include "stm32f1xx_hal.h"
#include "stdbool.h"

bool Push_Queue(uint8_t task_id);
uint8_t Pop_Queue(void);

#endif