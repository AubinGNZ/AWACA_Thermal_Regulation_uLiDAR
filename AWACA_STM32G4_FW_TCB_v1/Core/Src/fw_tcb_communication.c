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
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "main.h"
#include "fw_tcb_communication.h"

/* Variables ------------------------------------------------------------------*/
uint8_t Buffer_UART_RX[2] = {0};	// |Buffer containing data from UART RX
char Buffer_RX_Str[100] = {0};		// |Buffer used for conversion from byte to
									// |char
char *ID_value;

char *raw_frame;
char *cleared_frame;
char **organised_data;

char *pointer_begin_frame;
char *pointer_end_frame;

int index_begin_frame;
int index_end_frame;

/* RX Variables */
char ID_Device_RX[4] = {0};
float TCOB = 0;
float TCEB = 0;
char *STOB = 0;
char *STEB = 0;
char *DATA = 0;


float OB_T_C_Setpoint_OBC = 24.2;	// |Temperature Setpoint for OB sent by OBC

int OB_Flag_OT = 0;					// |Define if OT or UT alarms has been
int OB_Flag_UT = 0;					// |activated since last acquisition from OBC

int XOR = 0;
int checksum_RX = 0;
uint8_t flag_cks = 0;
int counter = 0;
int number_of_elemnts = 0;


/* Constant */
char begin_of_frame = '$';
char end_of_frame = '*';

/* Functions  ------------------------------------------------------------------*/

void Receiving_Data_OBC(void)
{

	Buffer_RX_Str[counter] = Buffer_UART_RX[0];

	counter++;

	if (isCompleteUartMessageReceived() || (counter > 99))
	    {
		raw_frame = malloc(100);

		for(int i = 0; i < 100 ; ++i) {
				raw_frame[i] = 0;
				raw_frame[i] = Buffer_RX_Str[i];
				Buffer_RX_Str[i] = 0;
			}

		cleared_frame = clearFrame(raw_frame);
		checksum_RX = raw_frame[index_end_frame + 1];
		counter = 0;
		memset(raw_frame,0,100);

		/* Checking Temperature Alarms States */

		Checking_Alarms_State_OB();



		/* Verifying Checksum */
		if (verifyCheckSum(cleared_frame) == 1) {
			flag_cks = false;
			Process_Data_OBC();
		}
		else {
			flag_cks = true;
			// Error Handler
		}
	}

	/* Reset UART Communication & wait for another byte*/
	HAL_UART_Receive_IT(&huart2, Buffer_UART_RX, 1);
}


void Process_Data_OBC(void)
{
	organised_data = str_split(cleared_frame, ';');
	referencingData();
}

void referencingData (void)
{
    char buffer[11];
    char *ID_value = calloc(5,1);
    char *value = calloc(5,1);

    // Arrange RX data to variables
    for(int i = 0 ; i < number_of_elemnts - 1; i++)
    {

        strncpy(buffer, *(organised_data + i), 10);
        strncpy(ID_value, &buffer[0], 4);
		if (!strcmp(ID_value, "OBC4")) {
			for(int i = 0 ; i < 4; i++) {
				ID_Device_RX[i] = ID_value[i];
			}
		}
		else if (!strcmp(ID_value, "TCOB")) {
			strncpy(value, &buffer[5], 4);
			TCOB = atof(value);
		}
		else if (!strcmp(ID_value, "TCEB")) {
			strncpy(value, &buffer[5], 4);
			TCEB = atof(value);
		}
		else if (!strcmp(ID_value, "STOB")) {
			strncpy(value, &buffer[5], 4);
			STOB = value;
			if (!strcmp(value, "1111")) {
			}
			else if (!strcmp(value, "0000")) {
			}
			else if (!strcmp(value, "PWSM")) {
			}
		}
		else if (!strcmp(ID_value, "STEB")) {
			strncpy(value, &buffer[5], 4);
			STEB = value;
			if (!strcmp(value, "1111")) {
			}
			else if (!strcmp(value, "0000")) {
			}
			else if (!strcmp(value, "PWSM")) {
			}
		}
		else if (!strcmp(ID_value, "DATA")) {
			strncpy(value, &buffer[5], 4);
			DATA = value;
		}
    }
}

uint8_t isCompleteUartMessageReceived(void)
{
	/* Checking if the last 2 char received are EOL characters */

	char EOL[2] = "\r\n";

	if (Buffer_RX_Str[counter - 2] == EOL[0] &&
    		Buffer_RX_Str[counter - 1] == EOL[1]) {
    	return true;
    }
    else {
    	return false;
    }

}


char *clearFrame (char *src)
{
	// Removing unnecessary bytes for data processing

	pointer_begin_frame = strchr(src, begin_of_frame);
    index_begin_frame = (int)(pointer_begin_frame - src);

    pointer_end_frame = strchr(src, end_of_frame);
    index_end_frame = (int)(pointer_end_frame - src);


    int cleared_frame_len = index_end_frame - index_begin_frame - 1;

    char *dst = malloc(cleared_frame_len + 1);
    for(int i = 0; i < cleared_frame_len ; ++i)
    {
         dst[i] = '0' ;
    }

    memcpy(dst,&src[index_begin_frame + 1], cleared_frame_len);
    dst[cleared_frame_len] = '\0';

    return (dst);
}


uint8_t verifyCheckSum(char *string)
{
    XOR = 0;
    // int checksum_RX = raw_frame[index_end_frame + 1];

    for (int i = 0; i < strlen(string); i++)
    {
        XOR = XOR ^ string[i];
    }

    if (checksum_RX == XOR) {
        return true;
    }
    else {
    	return false;
    }
}



/*   MISE EN PAUSE DE LA FONCTION
// |Function called inside UART Callback Func.
void Receiving_Data_OBC(void)
{
	for (int i = 0; i < 100; i++)
		{
			Buffer_RX_Str[i] = Buffer_UART_RX[i];
		}
	OB_T_C_Setpoint_OBC = atof(Buffer_RX_Str);
	Setting_Parameters_PicoLAS();



	HAL_UART_Receive_IT(&huart2, Buffer_UART_RX, 5);
	Checking_Alarms_State_OB();

}
*/

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

char** str_split(char* a_str, const char a_delim)
{
    char** result    = 0;
    size_t count     = 0;
    char* tmp        = a_str;
    char* last_comma = 0;
    char delim[2];
    delim[0] = a_delim;
    delim[1] = 0;

    /* Count how many elements will be extracted. */
    while (*tmp)
    {
        if (a_delim == *tmp)
        {
            count++;
            last_comma = tmp;
        }
        tmp++;
    }

    /* Add space for trailing token. */
    count += last_comma < (a_str + strlen(a_str) - 1);

    /* Add space for terminating null string so caller
       knows where the list of returned strings ends. */
    count++;

    result = malloc(sizeof(char*) * count);
    number_of_elemnts = count;

    if (result)
    {
        size_t idx  = 0;
        char* token = strtok(a_str, delim);

        while (token)
        {
            assert(idx < count);
            *(result + idx++) = strdup(token);
            token = strtok(0, delim);
        }
        assert(idx == count - 1);
        *(result + idx) = 0;
    }

    return result;
}

