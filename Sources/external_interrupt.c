/*
 * external_interrupt.c
#
 *
 *  Created on: Jul 10, 2015
 *      Author: Jun
 */
#include "external_interrupt.h"

void siu_external_irq_0(void){
	
	if( pin_read (34) == 1){
		start();

		for(int i = 0; i < 4; i++)
		   PIT_START_TIMER_CHANNEL(i);
		
		dbg_log("Timer on!");
	}

	SIU.ISR.R = 0x000000ff;
}


