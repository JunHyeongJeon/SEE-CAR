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

extern DRAW_MODE draw_mode;
extern int sona_check_cut_line;
extern bool is_school_zon_enable;
extern bool is_glcd_enable;
int pit_ch_2_ldval = 64000 * 3;
extern bool is_break_on;
extern int brake_value;
extern int SCHOOL_ZONE_SPEED_REF_LIMIT;


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
		if ( is_glcd_enable ){
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
				set_ref_speed(get_ref_speed() - 5);
				
				if ( get_ref_speed() < 0) 
					set_ref_speed(0);
			}else if (draw_mode == DrawSchoolZone){
				is_school_zon_enable = !is_school_zon_enable;
			}else if (draw_mode == DrawSiSet){
				// change HZ
				pit_ch_2_ldval = pit_ch_2_ldval - 16000;
				if (pit_ch_2_ldval <= 64000)pit_ch_2_ldval = 64000;
				PIT.CH[2].LDVAL.R = pit_ch_2_ldval;
			}else if ( draw_mode == DrawPSet){
				set_kp(get_kp()-100);
			}else if ( draw_mode == DrawISet){
				set_ki(get_ki()-10);
			}else if ( draw_mode == DrawDSet){
				set_kd(get_kd()-10);
			}else if( draw_mode == DrawBreakOn){
				is_break_on = !is_break_on;
			}else if( draw_mode == DrawSchoolZoneSpeed) {
				SCHOOL_ZONE_SPEED_REF_LIMIT -= 5;
			} else if( draw_mode == DrawTurnSpeed) {
				brake_value -= 5;
			}
		}
		if ( draw_mode == DrawGlcdSet){
			is_glcd_enable = !is_glcd_enable;
		}
		
		
		dbg_log("second button!");
	}
	else if (pin_read(37)  == 1){
		if( is_glcd_enable ){
			
			if( draw_mode == DrawCamera )
				toggle_glcd_draw_avg();
			
			else if ( draw_mode == DrawSona){
				sona_check_cut_line = sona_check_cut_line + 50;
			}else if ( draw_mode == DrawSpeed){
				set_ref_speed(get_ref_speed() + 5);	
			}else if ( draw_mode == DrawSchoolZone){
				is_school_zon_enable = !is_school_zon_enable;
			}else if (draw_mode == DrawSiSet){
				// change HZ
				pit_ch_2_ldval = pit_ch_2_ldval + 16000;
				if (pit_ch_2_ldval >= (64000 * 20)) pit_ch_2_ldval = 64000 * 20;
				PIT.CH[2].LDVAL.R = pit_ch_2_ldval;	
			}else if ( draw_mode == DrawPSet){
				set_kp(get_kp()+100);
			}else if ( draw_mode == DrawISet){
				set_ki(get_ki()+10);
			}else if ( draw_mode == DrawDSet){
				set_kd(get_kd()+10);
			}else if( draw_mode == DrawBreakOn){
				is_break_on = !is_break_on;
			}else if( draw_mode == DrawSchoolZoneSpeed) {
				SCHOOL_ZONE_SPEED_REF_LIMIT += 5;
			} else if( draw_mode == DrawTurnSpeed) {
				brake_value += 5;
			}
			
		}
		if ( draw_mode == DrawGlcdSet){
			is_glcd_enable = !is_glcd_enable;
		}
		dbg_log("third button!");		
	}
	else if(pin_read(14) == 1) {
		if ( is_glcd_enable )
			toggle_glcd_draw_mode();
		//draw_mode = 0;
	}
	if( draw_mode == DrawGlcdSet && is_glcd_enable){
		pin_write(71, 1);
	}
	else {
		pin_write(71, 0);
	}
	EnableExternalInterrupts();
	
	SIU.ISR.R = 0x000000ff;
}
