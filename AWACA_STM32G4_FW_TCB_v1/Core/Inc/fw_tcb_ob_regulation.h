/**
  ******************************************************************************
  * @file           : fw_tcb_ob_regulation.h
  * @brief          : OB Regulation module
  * @author			: Aubin IGNAZI
  * @project		: AWACA
  * @type 			: H file
  ******************************************************************************
  * In this file we will find the definition of the functions needed into the
  * process regulation of the optical box :
  ******************************************************************************
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FW_TCB_OB_REGULATION_H
#define __FW_TCB_OB_REGULATION_H


/* Includes ------------------------------------------------------------------*/


/* Private includes ----------------------------------------------------------*/


/* Exported types ------------------------------------------------------------*/
extern unsigned short OB_Reg_State;

extern unsigned long Buffer_ADC2[3];

extern uint16_t Raw_Data_ADC_NTC_FAN; 	// | 12-bits value from ADC conversion on PA_0
extern uint16_t Raw_Data_ADC_NTC_TEC; 	// | 12-bits value from ADC conversion on PA_1
extern uint16_t Raw_Data_TEC_Current; 	// | 12-bits value from ADC conversion on PA_5

extern float OB_T_C_Ext_NTC_OB;
extern float OB_T_C_Pic_NTC_OB;
extern float OB_I_A_Pic_TEC_OB;

extern unsigned int Duty_Cycle_PWM_Fan;

/* Exported constants --------------------------------------------------------*/


/* Exported macro ------------------------------------------------------------*/


/* Exported functions prototypes ---------------------------------------------*/
extern void Receiving_Data_OB(void);
extern void Data_Processing_OB(void);
extern void Setting_Parameters_PicoLAS(void);
extern void Control_Internal_Fan_OB(void);
/* Private defines -----------------------------------------------------------*/



#endif /* __FW_TCB_OB_REGULATION_H */
