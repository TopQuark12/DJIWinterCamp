/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

/**
  *********************** (C) COPYRIGHT 2018 DJI **********************
  * @update
  * @history
  * Version     Date              Author           Modification
  * V1.0.0      January-15-2018   ric.luo
  * @verbatim
  *********************** (C) COPYRIGHT 2018 DJI **********************
  */ 
 



#include "can_device.h"
#include "detect_task.h"
#include "sys.h"

/* 云台电机 */
moto_measure_t moto_pit;
moto_measure_t moto_yaw;
/* 拨弹电机 */
moto_measure_t moto_trigger;
/* 底盘电机 */
moto_measure_t moto_chassis[4];
/* 外围模块测试电机 */

/*Motor for imaginary robot arm*/
moto_measure_t moto_arm;							//CUSTOM

/**
  * @brief     CAN1 中断回调函数，在程序初始化时注册
  * @param     recv_id: CAN1 接收到的数据 ID
  * @param     data: 接收到的 CAN1 数据指针
  */
void can1_recv_callback(uint32_t recv_id, uint8_t data[])
{
  switch (recv_id)
  {
    case CAN_3508_M1_ID:
    {
      moto_chassis[0].msg_cnt++ <= 50 ? get_moto_offset(&moto_chassis[0], data) : \
      encoder_data_handle(&moto_chassis[0], data);
      err_detector_hook(CHASSIS_M1_OFFLINE);
    }
    break;
    case CAN_3508_M2_ID:
    {
      moto_chassis[1].msg_cnt++ <= 50 ? get_moto_offset(&moto_chassis[1], data) : \
      encoder_data_handle(&moto_chassis[1], data);
      err_detector_hook(CHASSIS_M2_OFFLINE);
    }
    break;

    case CAN_3508_M3_ID:
    {
      moto_chassis[2].msg_cnt++ <= 50 ? get_moto_offset(&moto_chassis[2], data) : \
      encoder_data_handle(&moto_chassis[2], data);
      err_detector_hook(CHASSIS_M3_OFFLINE);
    }
    break;
    case CAN_3508_M4_ID:
    {
      moto_chassis[3].msg_cnt++ <= 50 ? get_moto_offset(&moto_chassis[3], data) : \
      encoder_data_handle(&moto_chassis[3], data);
      err_detector_hook(CHASSIS_M4_OFFLINE);
    }
    break;
    case CAN_YAW_MOTOR_ID:
    {
      encoder_data_handle(&moto_yaw, data);
      err_detector_hook(GIMBAL_YAW_OFFLINE);
    }
    break;
    case CAN_PIT_MOTOR_ID:
    {
      encoder_data_handle(&moto_pit, data);
      err_detector_hook(GIMBAL_PIT_OFFLINE);
    }
    break;
    case CAN_TRIGGER_MOTOR_ID:
    {
      moto_trigger.msg_cnt++;
      moto_trigger.msg_cnt <= 10 ? get_moto_offset(&moto_trigger, data) : encoder_data_handle(&moto_trigger, data);
      err_detector_hook(TRIGGER_MOTO_OFFLINE);
    }
    break;
	case CAN_ARM_MOTOR_ID:																									//CUSTOM
    {
      moto_trigger.msg_cnt++;
      moto_trigger.msg_cnt <= 10 ? get_moto_offset(&moto_arm, data) : encoder_data_handle(&moto_arm, data);
      err_detector_hook(TRIGGER_MOTO_OFFLINE);
    }
    break;	

    default:
    {
    }
    break;
  }
}
  
/**
  * @brief     CAN2 中断回调函数，在程序初始化时注册
  * @param     recv_id: CAN2 接收到的数据 ID
  * @param     data: 接收到的 CAN2 数据指针
  */
void can2_recv_callback(uint32_t recv_id, uint8_t data[])
{
  switch (recv_id)
  {
    //case CAN2 device handle
    
    default:
    {
    }
    break;
  }
}

/**
  * @brief     获得电机初始偏差
  * @param     ptr: 电机参数 moto_measure_t 结构体指针
  * @param     data: 接收到的电机 CAN 数据指针
  */
static void get_moto_offset(moto_measure_t *ptr, uint8_t data[])
{
  ptr->ecd        = (uint16_t)(data[0] << 8 | data[1]);
  ptr->offset_ecd = ptr->ecd;
}

/**
  * @brief     计算电机的转速rmp 圈数round_cnt 
  *            总编码器数值total_ecd 总旋转的角度total_angle
  * @param     ptr: 电机参数 moto_measure_t 结构体指针
  * @param     data: 接收到的电机 CAN 数据指针
  */
static void encoder_data_handle(moto_measure_t *ptr, uint8_t data[])
{
  int32_t temp_sum = 0;
  
  ptr->last_ecd      = ptr->ecd;
  ptr->ecd           = (uint16_t)(data[0] << 8 | data[1]);

  ptr->speed_rpm     = (int16_t)(data[2] << 8 | data[3]);

  if (ptr->ecd - ptr->last_ecd > 5000)
  {
    ptr->round_cnt--;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd - 8192;
  }
  else if (ptr->ecd - ptr->last_ecd < -5000)
  {
    ptr->round_cnt++;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd + 8192;
  }
  else
  {
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd;
  }

  ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
  ptr->total_angle = ptr->total_ecd * 360 / 8192;
  
  
  ptr->rate_buf[ptr->buf_cut++] = ptr->ecd_raw_rate;
  if (ptr->buf_cut >= FILTER_BUF)
    ptr->buf_cut = 0;
  for (uint8_t i = 0; i < FILTER_BUF; i++)
  {
    temp_sum += ptr->rate_buf[i];
  }
  ptr->filter_rate = (int32_t)(temp_sum/FILTER_BUF);
}

/**
  * @brief     发送底盘电机电流数据到电调
  */
void send_chassis_moto_current(int16_t current[])
{
  static uint8_t data[8];
  
  data[0] = current[0] >> 8;
  data[1] = current[0];
  data[2] = current[1] >> 8;
  data[3] = current[1];
  data[4] = current[2] >> 8;
  data[5] = current[2];
  data[6] = current[3] >> 8;
  data[7] = current[3];
  
  write_can(CHASSIS_CAN, CAN_CHASSIS_ID, data);
}
void send_chassis_moto_zero_current(void)
{
  static uint8_t data[8];
  
  data[0] = 0;
  data[1] = 0;
  data[2] = 0;
  data[3] = 0;
  data[4] = 0;
  data[5] = 0;
  data[6] = 0;
  data[7] = 0;
  
  write_can(CHASSIS_CAN, CAN_CHASSIS_ID, data);
}

/**
  * @brief     发送云台电机电流数据到电调
  */
extern int16_t trigger_moto_current;
void send_gimbal_moto_current(int16_t yaw_current, int16_t pit_current)
{
  static uint8_t data[8];
  int16_t trigger_current = trigger_moto_current;
  
  data[0] = -yaw_current >> 8;
  data[1] = -yaw_current;
  data[2] = pit_current >> 8;
  data[3] = pit_current;
  data[4] = trigger_current >> 8;
  data[5] = trigger_current;
  data[6] = 0;
  data[7] = 0;
  
  write_can(GIMBAL_CAN, CAN_GIMBAL_ID, data);
}
void send_gimbal_moto_zero_current(void)
{
  static uint8_t data[8];
  
  data[0] = 0;
  data[1] = 0;
  data[2] = 0;
  data[3] = 0;
  data[4] = 0;
  data[5] = 0;
  data[6] = 0;
  data[7] = 0;
  
  write_can(GIMBAL_CAN, CAN_GIMBAL_ID, data);
}
/*Stops the arm motor*/
void send_arm_moto_zero_current(void)
{
  static uint8_t data[8];
  
  data[0] = 0;
  data[1] = 0;
  data[2] = 0;
  data[3] = 0;
  data[4] = 0;
  data[5] = 0;
  data[6] = 0;
  data[7] = 0;
  
  write_can(GIMBAL_CAN, CAN_GIMBAL_ID, data);
}
/*Moves the arm motor*/
void send_arm_moto_current(int16_t arm_current)
{
  static uint8_t data[8];
  
  data[0] = 0;
  data[1] = 0;
  data[2] = 0;
  data[3] = 0;
  data[4] = 0;
  data[5] = 0;
  data[6] = arm_current >> 8;
  data[7] = arm_current;
  
  write_can(GIMBAL_CAN, CAN_GIMBAL_ID, data);
}
void set_test_motor_current(int16_t test_moto_current[])
{
  static uint8_t data[8];
  
  data[0] = test_moto_current[0] >> 8;
  data[1] = test_moto_current[0];
  data[2] = 0;
  data[3] = 0;
  data[4] = 0;
  data[5] = 0;
  data[6] = 0;
  data[7] = 0;
  write_can(CHASSIS_CAN, CAN_CHASSIS_ID, data);
}
