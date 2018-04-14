#ifndef __EXECUTE_TASK_H__
#define __EXECUTE_TASK_H__

#define kP   	15
#define kI		0
#define kD		0

#include "stm32f4xx_hal.h"

void execute_task(const void* argu);

void test_moto_init(void);
void test_moto_control(void);

#endif

