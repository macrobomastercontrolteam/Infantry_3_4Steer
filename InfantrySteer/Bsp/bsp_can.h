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

#define CAN_CONTROL_ID_BASE   0x1ff
#define CAN_CONTROL_ID_EXTEND 0x2ff
#define STEER_MOTOR_COUNT     4

#define HALF_ECD_RANGE 4096
#define ECD_RANGE 8192
#define MOTOR_ECD_TO_RAD 0.000766990394f // (2*PI)/8192

typedef struct
{
    int16_t  set_voltage;
    uint16_t offset_ecd;
    uint16_t feedback_ecd;
    uint16_t target_ecd;
    // int16_t  rotor_speed;
    // int16_t  torque_current;
    // uint8_t  temperature;
}moto_info_t;

void can_user_init(void);
void CAN_cmd_steer_motors(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4);
#endif
