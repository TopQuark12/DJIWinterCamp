#include "execute_task.h"
#include "can_device.h"
#include "uart_device.h"
#include "pid.h"
#include "sys.h"

int16_t test_moto_speed = 0;
int16_t test_moto_current[1];
uint8_t test_sevo;

void test_moto_control(void)
{
	write_led_io(LED_IO1,LED_ON );
	
	
	
  test_moto_speed = rc.ch2 / RC_MAX_VALUE * MAX_WHEEL_RPM;
  
  //test_moto_current = test_moto_speed;
  
  
  test_moto_current[0] = pid_calc(&pid_wheel_spd[0], moto_chassis[0].speed_rpm, test_moto_speed);

	set_test_motor_current(test_moto_current);
	
	
	set_pwm_group_param(PWM_GROUP1,20000);
	start_pwm_output(PWM_IO1);
    
uint8_t sensor;
	
set_digital_io_dir (DIGI_IO1 ,IO_INPUT );
	
read_digital_io (DIGI_IO1 ,&sensor );
	
if(sensor==0 )
	set_pwm_param(PWM_IO1,2200);
else
	set_pwm_param(PWM_IO1,1500);


}


void test_moto_init(void)
{

 pid_init(&pid_pit, 7000, 0,
                  1, 0, 0); 
	  pid_init(&pid_wheel_spd[0], 7000, 0, 1, 0, 0);

}
