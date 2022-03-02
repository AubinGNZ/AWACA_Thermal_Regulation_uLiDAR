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
