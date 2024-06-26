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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#ifndef __BSP_CAN
#define __BSP_CAN

#include "can.h"
#include "arm_math.h"

#define CAN_CONTROL_ID_BASE   0x1ff
#define CAN_CONTROL_ID_EXTEND 0x2ff
#define STEER_MOTOR_COUNT     4
#define HIP_MOTOR_COUNT     4

#define ECD_RANGE 8192 // 360 degree
#define ECD_RANGE_180 4096 // 180 degree
#define ECD_RANGE_90 2048 // 90 degree
#define ECD_RANGE_45 1024 // 45 degree
#define MOTOR_ECD_TO_RAD (PI / (fp32)(ECD_RANGE_180))

typedef struct
{
	float set_torque;
	float output_angle;
    int16_t  set_voltage;
    uint16_t offset_ecd;
    uint16_t feedback_ecd;
    uint16_t target_ecd;
    int16_t  rotor_speed;
    // int16_t  torque_current;
    // uint8_t  temperature;
} motor_info_t;

typedef enum
{
	CHASSIS_ID_STEER_1 = 0, // right front
	CHASSIS_ID_STEER_2 = 1, // left front
	CHASSIS_ID_STEER_3 = 2, // left back
	CHASSIS_ID_STEER_4 = 3, // right back
	CHASSIS_ID_HIP_1 = 4, // right front
	CHASSIS_ID_HIP_2 = 5, // left front
	CHASSIS_ID_HIP_3 = 6, // left back
	CHASSIS_ID_HIP_4 = 7, // right back
	CHASSIS_ID_LAST,
} chassis_motor_ID_e;

extern uint8_t fLoadServoOn;
extern motor_info_t motor_info[CHASSIS_ID_LAST];

void can_user_init(void);
void CAN_cmd_steer_motors(uint8_t id_range, int16_t voltage1, int16_t voltage2, int16_t voltage3, int16_t voltage4);
uint8_t CAN_cmd_hip_motors(float torque1, float torque2, float torque3, float torque4);
HAL_StatusTypeDef encode_6012_multi_motor_torque_control(float torque1, float torque2, float torque3, float torque4);
#endif
