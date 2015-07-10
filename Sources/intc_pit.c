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
 * file                   : intc_pit.c
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
 * Brief Description      : This  file contains  the interrupt service routine  for the Periodic Interrupt Timer
 *
 ******************************************************************************** 
 *
 * Detail Description     : This file is generated when PIT(Periodic Interrupt
 *                         Timer) function is defined in INTC peripheral.This
 *                         file contains the Interrupt handlers routines for PIT.
 *                         In Interrupt handlers routine respective flags are cleared.
 *
 ******************************************************************************** 
 *
 *######################################################################
*/

 
 
/********************  Dependent Include files here **********************/

#include "intc_pit.h"

#include "gpio_drv.h"
#include "common.h"

#include "controls.h"
#include "line_scan.h"
#include "rappid_utils.h"
#include "core_ai.h"

#include "sona_sensor.h"

/************************* INTERRUPT HANDLERS ************************/

void line_sensing(void)
{
	line_scan();
	
	line_calc();	
	PIT.CH[PIT_LINE_SENSING_CHANNEL].TFLG.R = 0x00000001;
}
#define SONA_SENSING_ECHO_END_TIME 0x00001900

#define SONA_SENSING_RESPONSE_TIME 0x0004E200

#define SONA_SENSING_NEXT_TIME	   0x0009C400

void sona_sensing(void) {
	
	static enum {
		SonaEchoSent,
		SonaEchoEnded,
		SonaResponded,
	} sona_state = SonaResponded; // start
	
	int i = 0;
	char buf[10];
	
	switch(sona_state) {
		case SonaResponded: { // if respond ended restart it
			
			sona_sensor_send_echo();
			
			sona_state = SonaEchoSent;
			
			PIT.CH[PIT_SONA_SENSING_CHANNEL].LDVAL.R = SONA_SENSING_ECHO_END_TIME; // get it in next time
			
			break;
		}
		case SonaEchoSent: {
			
			sona_sensor_end_echo();
			
			sona_state = SonaEchoEnded;
			
			PIT.CH[PIT_SONA_SENSING_CHANNEL].LDVAL.R = SONA_SENSING_RESPONSE_TIME; // get it in next time
			
			break;
		}
		case SonaEchoEnded: {
			
			PIT.CH[PIT_SONA_SENSING_CHANNEL].LDVAL.R = SONA_SENSING_NEXT_TIME; // get it in next time
			
			i = sona_sensor_get_pulse_width(); // read it from emios
			
#ifdef DEBUG
			i_to_s_cnt(i, buf, 10);
#endif
			
			sona_state = SonaResponded;
			
			break;
		}
		default: {
			
			PIT.CH[PIT_SONA_SENSING_CHANNEL].LDVAL.R = SONA_SENSING_NEXT_TIME;
			
			sona_state = SonaResponded;
			
			break;
		}
	}
	
	PIT.CH[PIT_SONA_SENSING_CHANNEL].TFLG.R = 1; // End it
}

void ai_control(void) {
	
	dbg_log("Think~!");
	core_ai_think();
	PIT.CH[PIT_AI_THINK_CHANNEL].TFLG.R = 1;
}



/*
 *######################################################################
 *                           End of File
 *######################################################################
*/

