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
#include "core_ai.h"

extern enum {
	DrawCamera = 0,
	DrawSona,
	DrawSpeed,
	DrawSchoolZone
} draw_mode;
extern int sona_check_cut_line;
extern bool is_school_zon_enable;

void siu_external_irq_0(void){
	
	DisableExternalInterrupts();
	
	pin_write(41, 1);
	
	mdelay(30);
	
	pin_write(41, 0);
	
	if( pin_read (34) == 1){
		start();

		PIT_START_TIMER_CHANNEL(1); // sonar start
		PIT_START_TIMER_CHANNEL(2); // core ai start
	}
	
	else if( pin_read (35) == 1){
		if( draw_mode == DrawCamera){
			int new_line_draw = get_draw_line_select() + 1;
			
			if(new_line_draw >= LINE_CAMERA_VALUE_LINE_COUNT)
				new_line_draw = 0;
			
			set_glcd_draw_select(new_line_draw);
		}
		else if( draw_mode == DrawSona){
			sona_check_cut_line = sona_check_cut_line - 50;
			if ( sona_check_cut_line < 0) sona_check_cut_line =0;
			
		}else if( draw_mode == DrawSpeed){
			set_ref_speed(get_ref_speed() - 50);
			
			if ( get_ref_speed() < 0) 
				set_ref_speed(0);
		}else if (draw_mode == DrawSchoolZone){
			is_school_zon_enable = !is_school_zon_enable;
		}
		
		dbg_log("second button!");
	}
	else if (pin_read(37)  == 1){
		if( draw_mode == DrawCamera )
			toggle_glcd_draw_avg();
		
		else if ( draw_mode == DrawSona){
			sona_check_cut_line = sona_check_cut_line + 50;
		}
		else if ( draw_mode == DrawSpeed){
			set_ref_speed(get_ref_speed() + 50);
			
		}
		else if ( draw_mode == DrawSchoolZone){
			is_school_zon_enable = !is_school_zon_enable;
		}
		dbg_log("third button!");		
	}
	else if(pin_read(14) == 1) {
		toggle_glcd_draw_mode();
		//draw_mode = 0;
	}
	
	EnableExternalInterrupts();
	
	SIU.ISR.R = 0x000000ff;
}
