/*
 * external_interrupt.c
#
 *
 *  Created on: Jul 10, 2015
 *      Author: Jun
 */
#include "external_interrupt.h"
#include "intc_pit.h"
#include "line_scan.h"
#include "rappid_utils.h"

void siu_external_irq_0(void){
	
	DisableExternalInterrupts();
	
	pin_write(41, 1);
	
	mdelay(30);
	
	pin_write(41, 0);
	
	if( pin_read (34) == 1){
		start();

		for(int i = 0; i < 4; i++)
		   PIT_START_TIMER_CHANNEL(i);
		
		dbg_log("Timer on!");
	}
	else if( pin_read (35) == 1){
		
		int new_line_draw = get_draw_line_select() + 1;
		
		if(new_line_draw >= LINE_CAMERA_VALUE_LINE_COUNT)
			new_line_draw = 0;
		
		set_glcd_draw_select(new_line_draw);
		dbg_log("Change glcd!");
	}

	EnableExternalInterrupts();
	
	SIU.ISR.R = 0x000000ff;
}


