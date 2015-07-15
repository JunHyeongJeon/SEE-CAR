
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
#include "st7565.h"
#include "intc_pit.h"

/**********************  Function Prototype here *************************/

void main(void);

/*********************  Initialization Function(s) ************************/

extern void check_bluetooth();


void main(void)
{
	
	int i = 0;
	char buf[10];
	int y=0;

DisableExternalInterrupts();

/* ----------------------------------------------------------- */
/*	             System Initialization Function                  */
/* ----------------------------------------------------------- */
	sys_init_fnc();
  // SBC_Init_DBG();
	
   sys_log("Start");
   
   glcd_begin(0x18);	
   
   glcd_display();
   sdelay(1);
   glcd_clear_screen();
   
   dc_motor_init();
   encoder_init();
   
   line_scan_init();
   servo_motor_init();
   
   sona_sensor_init();
   
   PIT_START_TIMER_CHANNEL(PIT_AI_THINK_CHANNEL);
   PIT_START_TIMER_CHANNEL(PIT_CAMERA_CHANNEL);
   
   write_pin(PIN_BREAK_LIGHT, 0);
   write_pin(PIN_RIGHT_DIR_LIGHT, 0);
   write_pin(PIN_LEFT_DIR_LIGHT, 0);
   write_pin(PIN_4TH_LED, 0);

   set_glcd_draw_select(0);
   
   EnableExternalInterrupts();
   
   
	while(1) {
//		if(!is_started()) {
//			
//			line_scan();
////			line_calc();
//			//	    clear it before start
//			glcd_clear_screen();
//			
//			//	   	 proccess GLCD			
//			line_scan_draw_in_glcd(0);
//			
//			glcd_display();
//			
//			
//			mdelay(10);
//		}
		
		check_bluetooth();
	}
   
   sys_log("End");
}
