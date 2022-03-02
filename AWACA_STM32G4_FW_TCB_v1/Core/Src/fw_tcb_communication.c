/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : fw_tcb_communication.c
  * @brief          : Communication module
  * @author			: Aubin IGNAZI
  * @project		: AWACA
  * @type 			: C file
  ******************************************************************************
  * In this file we will find the definition of the function needed into the
  * process of communications :
  *
  * 1. Communications between the TCB and the OBC
  * 2. Communications between the TCB and PC for characterizations
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdlib.h>

#include "main.h"
#include "fw_tcb_communication.h"

/* Variables ------------------------------------------------------------------*/
uint8_t Buffer_UART_RX[8] = {0};	// |Buffer containing data from UART RX
char Buffer_RX_Str[8] = {0};		// |Buffer used for conversion from byte to
									// |char

float OB_T_C_Setpoint_OBC = 25;		// |Temperature Setpoint for OB sent by OBC

int OB_Flag_OT = 0;					// |Define if OT or UT alarms has been
int OB_Flag_UT = 0;					// |activated since last acquisition from OBC


/* Functions  ------------------------------------------------------------------*/

// |Function called inside UART Callback Func.
void Receiving_Data_OBC(void)
{
	for (int i = 0; i < 5; i++)
		{
			Buffer_RX_Str[i] = Buffer_UART_RX[i];
		}
	OB_T_C_Setpoint_OBC = atof(Buffer_RX_Str);
	Setting_Parameters_PicoLAS();

	/* Reset UART Communication & Checking Alarms State */

	HAL_UART_Receive_IT(&huart2, Buffer_UART_RX, 5);
	Checking_Alarms_State_OB();

}

void Checking_Alarms_State_OB(void)
{
	OB_Flag_OT = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);
	OB_Flag_UT = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);
}

void Setting_Alarms_State_OB(uint16_t Alarm_Index)
{
	/* OverTemperature Alarm has been triggered*/
	if (Alarm_Index == GPIO_PIN_10) {
		OB_Flag_OT = 1;
	}
	/* UnderTemperature Alarm has been triggered*/
	if (Alarm_Index == GPIO_PIN_12) {
		OB_Flag_UT = 1;
	}
}
