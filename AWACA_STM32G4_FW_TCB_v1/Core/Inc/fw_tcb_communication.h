/**
  ******************************************************************************
  * @file           : fw_tcb_communication.h
  * @brief          : Communication module
  * @author			: Aubin IGNAZI
  * @project		: AWACA
  * @type 			: H file
  ******************************************************************************
  * In this file we will find the definition of the function needed into the
  * process of communications :
  *
  * 1. Communications between the TCB and the OBC
  * 2. Communications between the TCB and PC for characterizations
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FW_TCB_COMMUNICATION_H
#define __FW_TCB_COMMUNICATION_H


/* Includes ------------------------------------------------------------------*/


/* Private includes ----------------------------------------------------------*/


/* Exported types ------------------------------------------------------------*/


extern uint8_t Buffer_UART_RX[8];

extern float Temp_Setpoint_OB_OBC;


/* Exported constants --------------------------------------------------------*/


/* Exported macro ------------------------------------------------------------*/


/* Exported functions prototypes ---------------------------------------------*/
void Receiving_Data(void);
void Checking_Alarms_State(void);
void Setting_Alarms_State(uint16_t);
/* Private defines -----------------------------------------------------------*/



#endif /* __FW_TCB_COMMUNICATION_H */
