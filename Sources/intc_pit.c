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
	//DisableExternalInterrupts();
	
	
    PIT.CH[PIT_LINE_SENSING_CHANNEL].TFLG.R = 0x00000001;
	line_scan();
	
	DisableExternalInterrupts();
	
	line_calc();
	EnableExternalInterrupts();
	
//	sys_log("======================================");
//	for(i = 0; i < 2; i++) {
//	
//		i_to_s_cnt(line_values_get_detected(i)[0], buf, 11);
//		sys_log(buf);
//	
//		i_to_s_cnt(line_values_get_detected(i)[1], buf, 11);
//		sys_log(buf);
//	} 
}
#define SONA_SENSING_ECHO_END_TIME 0x00001900

#define SONA_SENSING_RESPONSE_TIME 0x0004E200

#define SONA_SENSING_NEXT_TIME	   0x0000FA00

void sona_sensing(void) {
	
	static enum {
		SonaEchoSent,
		SonaEchoEnded,
		SonaResponded,
	} sona_state = SonaResponded; // start
	
	int i = 0;
	char buf[10];
	
	PIT.CH[PIT_LINE_SENSING_CHANNEL].TFLG.R = 1;
	
	switch(sona_state) {
	case SonaResponded: { // if respond ended restart it
		
		sona_sensor_send_echo();
		
		sona_state = SonaEchoSent;
		break;
	}
	case SonaEchoSent: {
		
		sona_sensor_end_echo();
		
		sona_state = SonaEchoEnded;
		break;
	}
	case SonaEchoEnded: {
		
		PIT.CH[PIT_LINE_SENSING_CHANNEL].LDVAL.R = SONA_SENSING_NEXT_TIME; // get it in next time
		
		i = sona_sensor_get_pulse_width(); // read it from emios
		i_to_s_cnt(i, buf, 10);
		print("Sona sensor : ");
		sys_log(buf);
		
		sona_state = SonaResponded;
		break;
	}
	default: {
		
		PIT.CH[PIT_LINE_SENSING_CHANNEL].LDVAL.R = SONA_SENSING_NEXT_TIME;
		
		sona_state = SonaResponded;
		break;
	}
	}
}

void ai_control(void) {
	
	sys_log("AI THINK");
	
	core_ai_think();
}

/*
 *######################################################################
 *                           End of File
 *######################################################################
*/

