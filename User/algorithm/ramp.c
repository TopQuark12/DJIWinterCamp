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
 



#include "ramp.h"

/* yaw 轴云台控制斜坡 */
ramp_t yaw_ramp = RAMP_GEN_DAFAULT;
/* pitch 轴云台控制斜坡 */
ramp_t pit_ramp = RAMP_GEN_DAFAULT;

/**
  * @brief     斜坡控制结构体初始化
  * @param[in] ramp: 斜坡数据结构体指针
  * @param[in] scale: 控制数据变化斜率
  */
void ramp_init(ramp_t *ramp, int32_t scale)
{
  ramp->count = 0;
  ramp->scale = scale;
}
/**
  * @brief     斜坡控制计算函数
  * @param[in] ramp: 斜坡数据结构体指针
  * @retval    斜坡控制计算输出
  */
float ramp_calc(ramp_t *ramp)
{
  if (ramp->scale <= 0)
    return 0;
  
  if (ramp->count++ >= ramp->scale)
    ramp->count = ramp->scale;
  
  ramp->out = ramp->count / ((float)ramp->scale);
  return ramp->out;
}
