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
char Buffer_RX_Str[8] = {0};		// |Buffer used for conversion to float

float Temp_Setpoint_OB_OBC = 25;	// |Temperature Setpoint for OB sent by OBC

int Flag_OT = 0;					// |Define if OT or UT alarms has been activated
int Flag_UT = 0;					// |since last acquisition from OBC
/* Functions  ------------------------------------------------------------------*/

// |Function called inside UART Callback Func.
void Receiving_Data()
{
	for (int i = 0; i < 5; i++)
		{
			Buffer_RX_Str[i] = Buffer_UART_RX[i];
		}
	Temp_Setpoint_OB_OBC = atof(Buffer_RX_Str);

	/* Reset UART Communication & Checking Alarms State */

	HAL_UART_Receive_IT(&huart2, Buffer_UART_RX, 5);
	Checking_Alarms_State();

}

void Checking_Alarms_State(void)
{
	Flag_OT = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);
	Flag_UT = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);
}

void Setting_Alarms_State(uint16_t Alarm_Index)
{
	/* OverTemperature Alarm has been triggered*/
	if (Alarm_Index == GPIO_PIN_10) {
		Flag_OT = 1;
	}
	/* UnderTemperature Alarm has been triggered*/
	if (Alarm_Index == GPIO_PIN_12) {
		Flag_UT = 1;
	}
}
