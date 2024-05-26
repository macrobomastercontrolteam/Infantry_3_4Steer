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
 
#include "bsp_can.h"
#include "string.h"

#define DISABLE_STEER_MOTOR 1
#define DISABLE_HIP_MOTOR 1

// reverse hip motor direction
#define REVERSE_3_HIP_MOTOR_DIRECTION 0
#define REVERSE_2_HIP_MOTOR_DIRECTION 0
#define REVERSE_4_HIP_MOTOR_DIRECTION 0
#define REVERSE_1_HIP_MOTOR_DIRECTION 0

#define MOTOR_6012_GEAR_RATIO 36.0f
#define MOTOR_6012_INPUT_TORQUE_TO_MAIN_CURRENT_RATIO 0.225146199f
#define MOTOR_6012_MAIN_CURRENT_TO_ROTOR_CURRENT_RATIO 0.212f
#define MOTOR_6012_CMD_TO_TORQUE_RATIO (1.0f / MOTOR_6012_GEAR_RATIO / MOTOR_6012_INPUT_TORQUE_TO_MAIN_CURRENT_RATIO / MOTOR_6012_MAIN_CURRENT_TO_ROTOR_CURRENT_RATIO / 33.0f * 2048.0f)
#define MOTOR_6012_BROADCAST_CMD_TO_TORQUE_RATIO (1.0f / MOTOR_6012_GEAR_RATIO / MOTOR_6012_INPUT_TORQUE_TO_MAIN_CURRENT_RATIO / MOTOR_6012_MAIN_CURRENT_TO_ROTOR_CURRENT_RATIO / 32.0f * 2000.0f)

/* CAN send and receive ID */
typedef enum
{
  // Custom IDs
  CAN_STEER_CONTROLLER_RX_ID = 0x112,
  CAN_CHASSIS_LOAD_SERVO_RX_ID = 0x113,
  // receives target speed for hip motors
  CAN_HIP_CONTROLLER_RX_ID = 0x114,
  // returns current position of hip motors
  CAN_HIP_CONTROLLER_TX_ID = 0x115,

  // steer motor tx
  CAN_CHASSIS_GM6020_TX_ID = 0x1FF,
  // hip motor tx
  CAN_HIP_MOTOR_SINGLECMD_TX_ID = 0x140,
	CAN_HIP_MOTOR_MULTICMD_TX_ID = 0x280,

  CAN_STEER1_RX_ID = 0x205,
  CAN_STEER2_RX_ID = 0x206,
  CAN_STEER3_RX_ID = 0x207,
  CAN_STEER4_RX_ID = 0x208,

  // 6012 motor as hip
	CAN_HIP1_RX_ID = 0x141,
	CAN_HIP2_RX_ID = 0x142,
	CAN_HIP3_RX_ID = 0x143,
	CAN_HIP4_RX_ID = 0x144,
} can_msg_id_e;

typedef enum
{
	CAN_6012_TORQUE_FEEDBACK_ID = 0xA1,
} can_msg_type_e;

motor_info_t motor_info[CHASSIS_ID_LAST];
uint8_t fLoadServoOn = 0;
uint8_t fSteerMotorEnabled = 0;
uint8_t fHipMotorEnabled = 0;
uint8_t abAllFF[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
CAN_TxHeaderTypeDef chassis_tx_message;
uint8_t chassis_can_send_data[8];

void decode_6020_motor_feedback(uint8_t *data, uint8_t bMotorId);
void decode_6012_motor_torque_feedback(uint8_t *data, uint8_t bMotorId);

/**
  * @brief  init can filter, start can, enable can rx interrupt
  * @retval None
  */
void can_user_init(void)
{
  CAN_FilterTypeDef  can_filter;

  can_filter.FilterBank = 0;                       // filter 0
  can_filter.FilterMode =  CAN_FILTERMODE_IDMASK;  // mask mode
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = 0;
  can_filter.FilterIdLow  = 0;
  can_filter.FilterMaskIdHigh = 0;
  can_filter.FilterMaskIdLow  = 0;                // set mask 0 to receive all can id
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0; // assign to fifo0
  can_filter.FilterActivation = ENABLE;           // enable can filter
  HAL_CAN_ConfigFilter(&hcan1, &can_filter);        // init can filter
  HAL_CAN_Start(&hcan1);                          // start can1
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // enable can1 rx interrupt

  // CAN filter setup refer to https://schulz-m.github.io/2017/03/23/stm32-can-id-filter/
  uint32_t filter_id = (CAN_STEER_CONTROLLER_RX_ID | CAN_CHASSIS_LOAD_SERVO_RX_ID | CAN_HIP_CONTROLLER_RX_ID);
  // Receive only the specified IDs
  uint32_t filter_mask = 0x1FFFFFFF & (~(CAN_STEER_CONTROLLER_RX_ID ^ CAN_CHASSIS_LOAD_SERVO_RX_ID)) & (~(CAN_STEER_CONTROLLER_RX_ID ^ CAN_HIP_CONTROLLER_RX_ID));
  can_filter.FilterIdHigh  = ((filter_id << 5)  | (filter_id >> (32 - 5))) & 0xFFFF; // STID[10:0] & EXTID[17:13]
  can_filter.FilterMaskIdHigh = ((filter_mask << 5)  | (filter_mask >> (32 - 5))) & 0xFFFF;
  can_filter.FilterMaskIdLow  = 0xFFFF;
  can_filter.SlaveStartFilterBank = 14;
  can_filter.FilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan2, &can_filter);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
  * @brief  can rx callback, get motor feedback info
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t             rx_data[8];
  uint8_t bMotorId = 0xFF;

  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); //receive can data

  if(hcan->Instance == CAN1)
  {
    // get motor index by can_id
	  switch (rx_header.StdId)
	  {
		  case CAN_STEER1_RX_ID:
		  case CAN_STEER2_RX_ID:
		  case CAN_STEER3_RX_ID:
		  case CAN_STEER4_RX_ID:
		  {
			  bMotorId = rx_header.StdId - CAN_STEER1_RX_ID + CHASSIS_ID_STEER_1;
			  decode_6020_motor_feedback(rx_data, bMotorId);
			  break;
		  }
      case CAN_HIP1_RX_ID:
      case CAN_HIP2_RX_ID:
      case CAN_HIP3_RX_ID:
      case CAN_HIP4_RX_ID:
      {
        if (rx_data[0] == CAN_6012_TORQUE_FEEDBACK_ID)
        {
          bMotorId = rx_header.StdId - CAN_HIP1_RX_ID + CHASSIS_ID_HIP_1;
          decode_6012_motor_torque_feedback(rx_data, bMotorId);
        }
        break;
      }
		  default:
		  {
			  break;
		  }
	  }
  }
  else if(hcan->Instance == CAN2)
  {
    // Note: CAN2 filter is used, so be careful with the mask config
    switch (rx_header.StdId)
    {
      case CAN_STEER_CONTROLLER_RX_ID:
      {
        fSteerMotorEnabled = (memcmp(rx_data, abAllFF, sizeof(abAllFF)) != 0);
        if (fSteerMotorEnabled)
        {
          for (bMotorId = 0; bMotorId < STEER_MOTOR_COUNT; bMotorId++)
          {
            motor_info[bMotorId].target_ecd = ((rx_data[2 * bMotorId] << 8) | rx_data[2 * bMotorId + 1]);
          }
        }
        break;
      }
      case CAN_HIP_CONTROLLER_RX_ID:
      {
        fHipMotorEnabled = (memcmp(rx_data, abAllFF, sizeof(abAllFF)) != 0);
        if (fHipMotorEnabled)
        {
          for (bMotorId = 0; bMotorId < HIP_MOTOR_COUNT; bMotorId++)
          {
            motor_info[bMotorId].target_ecd = ((rx_data[2 * bMotorId] << 8) | rx_data[2 * bMotorId + 1]);
          }
        }
        break;
      }
      case CAN_CHASSIS_LOAD_SERVO_RX_ID:
      {
        fLoadServoOn = rx_data[0];
        break;
      }
      default:
      {
        // Should not reach here
        Error_Handler();
        break;
      }
    }
  }
}

uint8_t CAN_cmd_hip_motors(float torque1, float torque2, float torque3, float torque4)
{
	uint8_t fValidInput = (((torque1 != torque1) || (torque2 != torque2) || (torque3 != torque3) || (torque4 != torque4)) == 0);
#if (DISABLE_HIP_MOTOR == 0)
  if ((fHipMotorEnabled == 0) || (fValidInput == 0))
#endif
  {
    torque1 = 0;
    torque2 = 0;
    torque3 = 0;
    torque4 = 0;
  }

#if REVERSE_3_HIP_MOTOR_DIRECTION
	torque3 *= -1.0f;
#endif

#if REVERSE_2_HIP_MOTOR_DIRECTION
	torque2 *= -1.0f;
#endif

#if REVERSE_4_HIP_MOTOR_DIRECTION
	torque4 *= -1.0f;
#endif

#if REVERSE_1_HIP_MOTOR_DIRECTION
	torque1 *= -1.0f;
#endif

	// 6012 motor as hip
	encode_6012_multi_motor_torque_control(torque1, torque2, torque3, torque4);
	return fValidInput;
}

HAL_StatusTypeDef encode_6012_multi_motor_torque_control(float torque1, float torque2, float torque3, float torque4)
{
	chassis_tx_message.StdId = CAN_HIP_MOTOR_MULTICMD_TX_ID;
	chassis_tx_message.IDE = CAN_ID_STD;
	chassis_tx_message.RTR = CAN_RTR_DATA;
	chassis_tx_message.DLC = 8;

	int16_t iqControl_1 = torque1 * MOTOR_6012_BROADCAST_CMD_TO_TORQUE_RATIO;
	int16_t iqControl_2 = torque2 * MOTOR_6012_BROADCAST_CMD_TO_TORQUE_RATIO;
	int16_t iqControl_3 = torque3 * MOTOR_6012_BROADCAST_CMD_TO_TORQUE_RATIO;
	int16_t iqControl_4 = torque4 * MOTOR_6012_BROADCAST_CMD_TO_TORQUE_RATIO;

	chassis_can_send_data[0] = *(uint8_t *)(&iqControl_1);
	chassis_can_send_data[1] = *((uint8_t *)(&iqControl_1) + 1);
	chassis_can_send_data[2] = *(uint8_t *)(&iqControl_2);
	chassis_can_send_data[3] = *((uint8_t *)(&iqControl_2) + 1);
	chassis_can_send_data[4] = *(uint8_t *)(&iqControl_3);
	chassis_can_send_data[5] = *((uint8_t *)(&iqControl_3) + 1);
	chassis_can_send_data[6] = *(uint8_t *)(&iqControl_4);
	chassis_can_send_data[7] = *((uint8_t *)(&iqControl_4) + 1);

  return HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, (uint32_t*) CAN_TX_MAILBOX0);
}

void decode_6012_motor_torque_feedback(uint8_t *data, uint8_t bMotorId)
{
	// int16_t iq_int = (data[3] << 8) | data[2];    // A
	int16_t v_int = (data[5] << 8) | data[4];     // deg/s
	uint16_t p_uint = ((data[7] << 8) | data[6]); // 16bit abs encoder

	// motor_info[bMotorId].torque = ((float)iq_int) / MOTOR_6012_CMD_TO_TORQUE_RATIO;
	// motor_info[bMotorId].velocity = ((float)v_int) / 36.0f / 180.0f * PI;
	motor_info[bMotorId].rotor_speed = v_int;
	motor_info[bMotorId].output_angle = ((float)p_uint) / (1 << 16) * 2.0f * PI;
	// motor_info[bMotorId].temperature = data[1];

#if (REVERSE_3_HIP_MOTOR_DIRECTION || REVERSE_2_HIP_MOTOR_DIRECTION || REVERSE_4_HIP_MOTOR_DIRECTION || REVERSE_1_HIP_MOTOR_DIRECTION)
	switch (bMotorId)
	{
#if REVERSE_3_HIP_MOTOR_DIRECTION
		case CHASSIS_ID_HIP_3:
#endif
#if REVERSE_2_HIP_MOTOR_DIRECTION
		case CHASSIS_ID_HIP_2:
#endif
#if REVERSE_4_HIP_MOTOR_DIRECTION
		case CHASSIS_ID_HIP_4:
#endif
#if REVERSE_1_HIP_MOTOR_DIRECTION
		case CHASSIS_ID_HIP_1:
#endif
		{
			// motor_info[bMotorId].torque *= -1.0f;
			// motor_info[bMotorId].velocity *= -1.0f;
			motor_info[bMotorId].rotor_speed *= -1;
			motor_info[bMotorId].output_angle *= -1.0f;
			break;
		}
		default:
		{
			break;
		}
	}
#endif

	// // Capstone mechanical frame settings
	// // zeros of hip motors are calibrated to limiter position for accuracy
	// const float hip_limiter_offset_angle = 7.0f / 180.0f * PI;
	// switch (bMotorId)
	// {
	// 	case CHASSIS_ID_HIP_3:
	// 	case CHASSIS_ID_HIP_2:
	// 	case CHASSIS_ID_HIP_1:
	// 	{
	// 		motor_info[bMotorId].output_angle -= hip_limiter_offset_angle;
	// 		break;
	// 	}
	// 	case CHASSIS_ID_HIP_4:
	// 	{
	// 		motor_info[bMotorId].output_angle += hip_limiter_offset_angle;
	// 		break;
	// 	}
	// 	default:
	// 	{
	// 		break;
	// 	}
	// }
}

void decode_6020_motor_feedback(uint8_t *data, uint8_t bMotorId)
{
	motor_info[bMotorId].feedback_ecd    = ((data[0] << 8) | data[1]);
  motor_info[bMotorId].rotor_speed    = ((data[2] << 8) | data[3]);
  // motor_info[bMotorId].torque_current = ((data[4] << 8) | data[5]);
  // motor_info[bMotorId].temperature    =   data[6];
}

/**
  * @brief  send motor control message through can bus
  * @param  motor voltage 1,2,3,4 or 5,6,7
  * @retval None
  */
void CAN_cmd_steer_motors(uint8_t id_range, int16_t voltage1, int16_t voltage2, int16_t voltage3, int16_t voltage4)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t tx_data[8];

  tx_header.StdId = (id_range == 0) ? (CAN_CONTROL_ID_BASE) : (CAN_CONTROL_ID_EXTEND);
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 8;

#if (DISABLE_STEER_MOTOR == 0)
  if (fSteerMotorEnabled == 0)
#endif
  {
    voltage1 = 0;
    voltage2 = 0;
    voltage3 = 0;
    voltage4 = 0;
  }

  tx_data[0] = voltage1 >> 8;
  tx_data[1] = voltage1;
  tx_data[2] = voltage2 >> 8;
  tx_data[3] = voltage2;
  tx_data[4] = voltage3 >> 8;
  tx_data[5] = voltage3;
  tx_data[6] = voltage4 >> 8;
  tx_data[7] = voltage4;
  HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}
