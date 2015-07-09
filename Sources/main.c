/*
 *######################################################################
 *                                RAppIDJDP
 *           Rapid Application Initialization and Documentation Tool
 *                         Freescale Semiconductor Inc.
 *
 *######################################################################
 *
 * Project Name           : FSK_LED_Example
 *
 * Project File           : FSK_LED_Example.rsp
 *
 * Revision Number        : 1.0
 *
 * Tool Version           : 1.2.1.5
 *
 * file                   : main.c
 *
 * Target Compiler        : Codewarrior
 *
 * Target Part            : MPC5606B
 *
 * Part Errata Fixes      : none
 *
 * Project Last Save Date : 28-Oct-2013 16:04:30
 *
 * Created on Date        : 28-Oct-2013 16:04:31
 *
 * Brief Description      : This file contains main() function call.
 *
 ******************************************************************************** 
 *
 * Detail Description     : This file contains main() routine which calls system
 *                         initialization routine and interrupt enable routine if defined.
 *
 ******************************************************************************** 
 *
 *######################################################################
*/

 
 
/********************  Dependent Include files here **********************/

#include "rappid_ref.h"
#include "rappid_utils.h"
#include "sys_init.h"
#include "CANapi.h"
#include "sbc_hld.h"
#include "gpio_drv.h"
#include "pot_hld.h"
#include "photo_sensor_hld.h"
//#include "freemaster.h"
#include "common.h"
#include "UART_drv_api.h"
#include "servo_motor.h"
#include "dc_motor.h"
#include "encoder.h"
#include "line_scan.h"
#include "sona_sensor.h"

/**********************  Function Prototype here *************************/

void title(void);
void main(void);
void ProcessCAN(void);
void ProcessADC(void);

/**********************  Global variables here *************************/
/* CAN messages to transmit */
unsigned char msgOKCAN[8] = {1,1,0,0,0,0,0,0};
unsigned char msgErrorCAN[8] = {1,0xFF,0,0,0,0,0,0};

uint16_t potValue;		/* Potentiometer ADC input value*/
uint16_t photoSensorValue;	/* Photo sensor ADC input value */

/*********************  Initialization Function(s) ************************/




void main(void)
{
	
	int i = 0;
	char buf[10];
	
/* ----------------------------------------------------------- */
/*	             System Initialization Function                  */
/* ----------------------------------------------------------- */
	sys_init_fnc();
   /* Initialize SBC */
   SBC_Init_DBG();   

   /* FreeMASTER internal variables initialization */
   //FMSTR_Init();
   
/********* Enable External Interrupt *********/
   EnableExternalInterrupts();
   
   /* Turn off LEDs */  
   GPIO_SetState(68, 1);
   GPIO_SetState(69, 1);
   GPIO_SetState(70, 1);
   GPIO_SetState(71, 1);

   sys_log("Start");
   
   /* Initialize CAN filter */ 
   SetCanRxFilter(1, 0, 0);
//   ks0108_init_state();
//   ks0108_clear_screen(KS0108_WHITE);
   
//   line_scan_init();
//   sys_log("scan line");
//   line_scan();
//   
//   for(i = 0; i < LINE_CAMERA_PIXEL_CONUT; i++) {
//	   i_to_s(line_values[0][i], buffer);
//	   sys_log(buffer);
//   }
//   
//   sys_log("print end");

   dc_motor_init();
   encoder_init();
   
   line_scan_init();
   servo_motor_init();
   
   sona_sensor_init();
   
   while(1) {
	   	   
	  dc_motor_left_set_duty_rate(150);
	  dc_motor_right_set_duty_rate(150);
	  i = encoder_read_left();
	  i_to_s_cnt(i, buf, 10);
	  sys_log(buf);
	  
	  i = encoder_read_right();
	  i_to_s_cnt(i, buf, 10);
	  sys_log(buf);
	  	
	  sys_log("==========================");
//	delay(10);
	   
	  line_scan();
	  line_calc();
	  for(i = 0; i < 2; i++) {
	
		i_to_s_cnt(line_values_get_detected(i)[0], buf, 11);
		sys_log(buf);
	
		i_to_s_cnt(line_values_get_detected(i)[1], buf, 11);
		sys_log(buf);
	  }	  
	 
	servo_motor_move(20);
	  mdelay(1);
	   
//	   ProcessADC();
   }
   
   sys_log("End");
}

/******************************************************************************
*   Function: ProcessCAN
*
*   Description: Process CAN messages
*
******************************************************************************/
void ProcessCAN(void)
{
	can_msg_struct msgCanRX;
	
	if (CanRxMbFull(0) == 1)		/* Check if CAN message received */
	{
	    msgCanRX = CanRxMsg(0);	
	    if (msgCanRX.data[0] == 0)	/* If first data byte is 0, turn off LED3 and send positive response */
	    {
	    	GPIO_SetState(70, 1);
	    	CanTxMsg (2, 1, 8, (uint8_t *)msgOKCAN, 0);
	    }
	    else if (msgCanRX.data[0] == 1)/* If first data byte is 1, turn on LED3 and send positive response */
	    {
	    	GPIO_SetState(70, 0);
	    	CanTxMsg (2, 1, 8, (uint8_t *)msgOKCAN, 0); 
	    }
	    else			/* If first data byte is not 0 or 1, send a negative response */
	    {
	    	CanTxMsg (2, 1, 8, (uint8_t *)msgErrorCAN, 0);
	    }	    		
	}
}

/******************************************************************************
*   Function: ProcessADC
*
*   Description: Processes Potentiometer and Photo sensor ADC inputs 
*
******************************************************************************/

void ProcessADC(void)
{
	potValue = Pot_Get_Value();
	if(potValue <= 250) 
	{
		GPIO_SetState(68, 0);
		GPIO_SetState(69, 1);
		GPIO_SetState(70, 1);
		GPIO_SetState(71, 1);
	}
	else if (potValue > 250 && potValue <=500)
	{
		GPIO_SetState(68, 1);
		GPIO_SetState(69, 0);
		GPIO_SetState(70, 1);
		GPIO_SetState(71, 1);
	}
	else if (potValue > 500 && potValue <=750)
	{
		GPIO_SetState(68, 1);
		GPIO_SetState(69, 1);
		GPIO_SetState(70, 0);
		GPIO_SetState(71, 1);
	}
	else
	{
		GPIO_SetState(68, 1);
		GPIO_SetState(69, 1);
		GPIO_SetState(70, 1);
		GPIO_SetState(71, 0);
	}
	
//	photoSensorValue = Photo_Sensor_Get_Value();
//	if(photoSensorValue <= 500) /* If Photo sensor input is <= 500 turn on LED2, otherwise turn off LED2 */
//	{
//		GPIO_SetState(69, 0);
//	}
//	else
//	{
//		GPIO_SetState(69, 1);
//	}	
} 

 
/*
 *######################################################################
 *                           End of File
 *######################################################################
*/

