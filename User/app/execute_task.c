#include "execute_task.h"
#include "can_device.h"
#include "uart_device.h"
#include "cmsis_os.h"
#include "calibrate.h"
#include "pid.h"
#include "sys.h"


extern moto_measure_t moto_arm;
pid_t arm_pid;
int current_pos;
float current_angle;
int target_pos;
int control_variable;
int abs_error;
int dead_zone = 3;

static uint8_t abs_limit(float *a, float ABS_MAX)
{
  if (*a > ABS_MAX) {
	  *a = ABS_MAX;
	  return 1;
  }
      
  if (*a < -ABS_MAX){
	  *a = -ABS_MAX;
	  return 1;
  }
  
  return 0;
  
}

float arm_pid_calc(pid_t *pid, float get, float set)
{
	
  uint8_t maxed = 0;
  pid->get = get;
  pid->set = set;
  pid->err[NOW] = set - get; 
  
  pid->pout = pid->p * pid->err[NOW];
  pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST]);

  pid->out = pid->pout + (pid->i * (pid->err[NOW] + pid->iout)) + pid->dout;
  maxed = abs_limit(&(pid->out), pid->max_output);
	
  pid->iout = maxed ? 0 : pid->iout + pid->err[NOW];
	
  abs_limit(&(pid->iout), pid->integral_limit);

  pid->err[LAST] = pid->err[NOW];
  
  return pid->out;
}

//C620 + 3508 		(20000, 50000, 10, 0.02, 600)
//C610 + 2006		(10000, 20000, 10, 0.2, 2000)

void execute_task(const void* argu)
{

	pid_init(&arm_pid, 20000, 50000, 10, 0.02, 600);
	target_pos = 0;

  while (1)
  {	
	current_pos = moto_arm.total_ecd;
	current_angle = current_pos * 360.0 / 8192;
	abs_error = (current_angle - target_pos > 0) ? current_angle - target_pos : target_pos - current_angle;
	if (abs_error > dead_zone) {
		control_variable = arm_pid_calc(&arm_pid, current_angle, target_pos);
		send_arm_moto_current(control_variable);	
	}	
    osDelay(1);
  }
}
