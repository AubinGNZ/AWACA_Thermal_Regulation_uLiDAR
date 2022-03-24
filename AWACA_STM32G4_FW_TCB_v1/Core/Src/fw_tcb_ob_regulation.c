/**
  ******************************************************************************
  * @file           : fw_tcb_ob_regulation.c
  * @brief          : OB Regulation module
  * @author			: Aubin IGNAZI
  * @project		: AWACA
  * @type 			: C file
  ******************************************************************************
  * In this file we will find the definition of the functions needed into the
  * process regulation of the optical box :
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "fw_tcb_ob_regulation.h"
#include "main.h"


/* Variables ------------------------------------------------------------------*/
uint16_t Raw_Data_ADC_NTC_FAN; 	// | 12-bits value from ADC conversion on PA_0
uint16_t Raw_Data_ADC_NTC_TEC; 	// | 12-bits value from ADC conversion on PA_1
uint16_t Raw_Data_TEC_Current; 	// | 12-bits value from ADC conversion on PA_5

static float T_25_K = 298.15;  	// | Kelvin
static float T_0_K = 273.15;	// | Kelvin
static float B_NTC_Param = 3954;// | Kelvin

/* External variables */
float OB_T_C_Ext_NTC_OB = 0;	// | float value of ext. temperature NTC in OB
float OB_T_C_Pic_NTC_OB = 0;	// | float value of PicoLAS temperature NTC in OB
float OB_I_A_Pic_TEC_OB = 0;	// | float value of PicoLAS current through TEC

unsigned long Buffer_ADC2[3] = {0};

unsigned short OB_Reg_State = 1;	// | ON/OFF of OB Regulation
unsigned int Duty_Cycle_PWM_Fan = 0;

float V_out_ADC_Ext_NTC_OB;
float V_out_ADC_Pic_NTC_OB;
float V_out_ADC_Pic_Cur_OB;

float OB_T_32b_Setpoint_OBC;
float V_Ref_PicoLAS = 1.5;

/* Functions  ------------------------------------------------------------------*/
void Receiving_Data_OB(void)
{
	Raw_Data_ADC_NTC_TEC = Buffer_ADC2[0];	// | 12-bits value from ADC conversion on PA_1
	Raw_Data_TEC_Current = Buffer_ADC2[1];
	Raw_Data_ADC_NTC_FAN = Buffer_ADC2[2];	// | 12-bits value from ADC conversion on PA_0
/*
|Start ADC 1 : eq. voltage of external NTC
HAL_ADC_Start(&hadc1);
HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
Raw_Data_ADC_NTC_FAN = HAL_ADC_GetValue(&hadc1);
HAL_ADC_Stop(&hadc1);

|Start ADC 2 : eq. voltage of PicoLAS NTC
ADC2_Select_CH2(); 									 Select Channel 2
HAL_ADC_Start(&hadc2);
HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
Raw_Data_ADC_NTC_TEC = HAL_ADC_GetValue(&hadc2);
HAL_ADC_Stop(&hadc2);

|Start ADC 2 : eq. current through Peltier
ADC2_Select_CH13(); 								 Select Channel 13
HAL_ADC_Start(&hadc2);
HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
Raw_Data_TEC_Current = HAL_ADC_GetValue(&hadc2);
HAL_ADC_Stop(&hadc2);
*/


}

void Data_Processing_OB(void)
{

	static float V_supply = 3.3;

	/* Conversion from ADC 1 Voltage to Temperature (°C) */
	V_out_ADC_Ext_NTC_OB = Raw_Data_ADC_NTC_FAN * (3.36 / 4095);
	OB_T_C_Ext_NTC_OB = ((T_25_K * B_NTC_Param) / (T_25_K *
						log(V_out_ADC_Ext_NTC_OB / (V_supply -
						V_out_ADC_Ext_NTC_OB)) + B_NTC_Param)) - T_0_K;

	/* Conversion from ADC 2 Ch. 2 Voltage to Temperature (°C) */
	V_out_ADC_Pic_NTC_OB = Raw_Data_ADC_NTC_TEC * (3.36 / 4095);
	OB_T_C_Pic_NTC_OB = B_NTC_Param / ((B_NTC_Param / T_25_K)
						+ log((V_out_ADC_Pic_NTC_OB - 4.5)/(-4 * 1.5) /
						( 1 - (V_out_ADC_Pic_NTC_OB - 4.5)/(-4 * 1.5))))
						- T_0_K;

	/* Conversion from ADC 2 Ch. 13 Voltage to Current (A) */
	V_out_ADC_Pic_Cur_OB = Raw_Data_TEC_Current * (3.3/4095) ;
	OB_I_A_Pic_TEC_OB = (V_out_ADC_Pic_Cur_OB - 1.5) / 0.4 ;

}

void Setting_Parameters_PicoLAS(void)
{

	/* Set SHDN to activate OB regulation */
	if(OB_Reg_State == 1) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
	}
	else if (OB_Reg_State == 0) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	}
	else {

	}

	/* Set DAC to define the Temperature Setpoing for OB Regulation */

	OB_T_32b_Setpoint_OBC = V_Ref_PicoLAS*(exp(B_NTC_Param*((1 /
							(OB_T_C_Setpoint_OBC + T_0_K) - (1 / T_25_K)))) /
							(1 + exp(B_NTC_Param * ((1 / (OB_T_C_Setpoint_OBC
							+ T_0_K) - (1 / T_25_K))))))*(0xfff+1)/3.3;

	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
							OB_T_32b_Setpoint_OBC);
}

void Control_Internal_Fan_OB(void)
{
	if(OB_T_C_Ext_NTC_OB < 25)
	{
		Duty_Cycle_PWM_Fan = 56000; // Speed 1 (75% Speed)
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, RESET);
		UpdatePWMDutyCycle();
	}
	else
	{
		Duty_Cycle_PWM_Fan = 60000; // Speed 2 (100% Speed)
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, SET);
		UpdatePWMDutyCycle();
	}
}
