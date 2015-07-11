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

#include "st7565.h"

#include "sona_sensor.h"

/************************* INTERRUPT HANDLERS ************************/


void check_bluetooth();

#ifndef USE_CAM_1

static bool is_started = false;
#endif

#define SONA_SENSING_ECHO_END_TIME 0x00001900

#define SONA_SENSING_RESPONSE_TIME 0x0004E200

#define SONA_SENSING_NEXT_TIME	   0x0009C400

static int line_draw_select = 0;

void line_sensing(void)
{
//#ifdef DEBUG
//	check_bluetooth();
//#endif
	line_scan();
	
	line_calc();
	
	PIT_COMMIT_TIMER(PIT_LINE_SENSING_CHANNEL);
}

void sona_sensing(void) {
	
	static enum {
		SonaEchoSent,
		SonaEchoEnded,
		SonaResponded,
	} sona_state = SonaResponded; // start
	
	int i = 0;
	char buf[10];
	
//#ifdef DEBUG
//	check_bluetooth();
//#endif
	
	switch(sona_state) {
		case SonaResponded: { // if respond ended restart it
			
			sona_sensor_send_echo();
			
			sona_state = SonaEchoSent;
			
			// wait timing for signal end
			PIT_SET_TIMER(PIT_SONA_SENSING_CHANNEL, SONA_SENSING_ECHO_END_TIME);

			break;
		}
		case SonaEchoSent: {
			
			sona_sensor_end_echo();
			
			sona_state = SonaEchoEnded;
			
			// wait for response time
			PIT_SET_TIMER(PIT_SONA_SENSING_CHANNEL, SONA_SENSING_RESPONSE_TIME);

			break;
		}
		case SonaEchoEnded: {
			
			// get it in next time
			PIT_SET_TIMER(PIT_SONA_SENSING_CHANNEL, SONA_SENSING_NEXT_TIME);
						
			i = sona_sensor_get_pulse_width(); // read it from emios
			
#ifdef DEBUG
			i_to_s_cnt(i, buf, 10);
#endif
			
			sona_state = SonaResponded;
			
			break;
		}
		default: {
			
			PIT_SET_TIMER(PIT_SONA_SENSING_CHANNEL, SONA_SENSING_NEXT_TIME);
			
			sona_state = SonaResponded;
			
			break;
		}
	}
	
	PIT_COMMIT_TIMER(PIT_SONA_SENSING_CHANNEL); // End it
}

void ai_control(void) {
	
	dbg_log("Think~!");
	
	DisableExternalInterrupts();
	
	line_scan();
		
	line_calc();
		
	
#ifdef DEBUG
	glcd_clear_screen();
		
		// proccess GLCD
		
		line_scan_draw_in_glcd(line_draw_select);
		
		glcd_display();
		
		
		// commit timer
		PIT_COMMIT_TIMER(PIT_UTILITY_CHANNEL);
#endif
	
	check_bluetooth();
	
	core_ai_think();
	
	PIT_COMMIT_TIMER(PIT_AI_THINK_CHANNEL);
	EnableExternalInterrupts();
}

void utility_proccess() {
	
//	DisableExternalInterrupts();
	
	check_bluetooth();
	
	// clear it before start
//	glcd_clear_screen();
	
	// proccess GLCD
	
//	line_scan_draw_in_glcd(line_draw_select);
	
//	glcd_display();
	
	
	// commit timer
	PIT_COMMIT_TIMER(PIT_UTILITY_CHANNEL);
	
//	EnableExternalInterrupts();
}

void check_bluetooth() {
	
	UartRxFillBuf();
	
	if (UartRxBufEmpty() != 1) {
		
		unsigned char data = UartRxDataByte();
		
		switch(data) {
		case 's': {
			
#ifdef USE_CAM_1
			if(!is_started()) {
				
				start();
				
				for(int i = 0; i < 4; i++)
				   PIT_START_TIMER_CHANNEL(i);
			}
			
#else
			if(!is_started) {
				PIT_START_TIMER_CHANNEL(PIT_AI_THINK_CHANNEL);
				PIT_STOP_TIMER_CHANNEL(PIT_UTILITY_CHANNEL);
			}
			else {
				PIT_STOP_TIMER_CHANNEL(PIT_AI_THINK_CHANNEL);
				PIT_START_TIMER_CHANNEL(PIT_UTILITY_CHANNEL);
			}
			
			is_started = !is_started;
#endif
			
			break;
		}
		case 'g':
			
			line_draw_select++;
			
			if(line_draw_select >= LINE_CAMERA_VALUE_LINE_COUNT)
				line_draw_select = 0;
			
			break;
		case 'a':
			GPIO_SetState(69, 0);
			break;
		case 'b':
			GPIO_SetState(69, 1);
			break;
		}
	}
}

/*
 *######################################################################
 *                           End of File
 *######################################################################
*/

