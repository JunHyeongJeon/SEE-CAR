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

#include "line_scan.h"
#include "rappid_utils.h"
#include "core_ai.h"

#include "st7565.h"

#include "sona_sensor.h"

#include "servo_motor.h"
#include "draw_in_glcd.h"
/************************* INTERRUPT HANDLERS ************************/


#define DEBUG_FUNC(VAR_NAME, VAL) print(VAR_NAME);print(": "); i_to_s_cnt(VAL, buf, 10); sys_log(buf);

void check_bluetooth();


#ifndef USE_CAM_1

static bool is_started = false;
#endif

#define SONA_SENSING_ECHO_END_TIME 0x00001900

#define SONA_SENSING_RESPONSE_TIME 0x0004E200

#define SONA_SENSING_NEXT_TIME	   0x0009C400

extern int theta;
extern int brake_value;
bool is_glcd_enable = true;
static bool boost_up_mode = false;
extern DRAW_MODE draw_mode = DrawCamera;

static bool draw_avg = false;
void caution_light_handler(void)
{
	
	static bool light_on = false;
	
	if(light_on) {
		write_pin(PIN_LEFT_DIR_LIGHT, 0);
		write_pin(PIN_RIGHT_DIR_LIGHT, 0);
	}
	else {
		write_pin(PIN_LEFT_DIR_LIGHT, 1);
		write_pin(PIN_RIGHT_DIR_LIGHT, 1);
		
	}
	
	light_on = !light_on;
	
	PIT_COMMIT_TIMER(PIT_CAUTION_LIGHT_CHANNEL);
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
			
//#ifdef DEBUG
//			i_to_s_cnt(i, buf, 10);
//			sys_log(buf);
//#endif
			
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
	
//	DisableExternalInterrupts();
//	
//	if(is_started()) {	
//		EnableExternalInterrupts();
//	}
#ifdef DEBUG
	
//	if(!is_started()) {
//		i_to_s_cnt(get_draw_line_select(), buf, 2);
//		drawstring(0, 60, buf);
	if(is_glcd_enable && !is_started()){	
		if(draw_mode == DrawCamera)
			line_scan_draw_in_glcd(get_draw_line_select(), draw_avg);
		else if(draw_mode == DrawSona) {
			sona_sensor_draw_in_glcd();
		}else if ( draw_mode == DrawSpeed){
			speed_draw_in_glcd();
		}else if ( draw_mode == DrawSchoolZone){
			school_zone_draw_in_glcd();
		}else if ( draw_mode == DrawGlcdSet){
			glcd_set_draw_in_glcd();
		}else if ( draw_mode == DrawSiSet){
			glcd_si_draw_in_glcd();
		}else if ( draw_mode == DrawPSet){
			glcd_p_draw_in_glcd();
		}else if ( draw_mode == DrawISet){
			glcd_i_draw_in_glcd();
		}else if ( draw_mode == DrawDSet){
			glcd_d_draw_in_glcd();
		}else if( draw_mode == DrawBreakOn){
			draw_on_break_in_glcd();
		}else if( draw_mode == DrawSchoolZoneSpeed) {
			draw_schoolzone_speed();
		} else if( draw_mode == DrawTurnSpeed) {
			draw_break_sp_in_glcd();
		}
		
	}
//	}
#endif
	
	check_bluetooth();
	
	if(is_started()) {
		core_ai_think();
	}
	
	PIT_COMMIT_TIMER(PIT_AI_THINK_CHANNEL);
}

void camera_proccess() {
	
	DisableExternalInterrupts();
	
	line_scan();
			
	if(is_started()) {		
		line_calc();
		
	}
	
	EnableExternalInterrupts();
	
	PIT_COMMIT_TIMER(PIT_CAMERA_CHANNEL);
}

extern int SCHOOL_ZONE_SPEED_REF_LIMIT;

void check_bluetooth() {
	
	char buf[10];
	
	UartRxFillBuf();
	static int servo = 512;
	int ref_speed;
	
	if (UartRxBufEmpty() != 1) {
		
		unsigned char data = UartRxDataByte();
		
		switch(data) {
#ifdef USE_CAM_1
		case 'f':
			boost_up_mode = true;
		case 'p':				
			for(int i = 0; i < 4; i++)
			   PIT_START_TIMER_CHANNEL(i);

			break;
#endif
		case 'g':
			
			line_draw_select++;
			
			if(line_draw_select >= LINE_CAMERA_VALUE_LINE_COUNT)
				line_draw_select = 0;
			
			break;
		case ']':
			servo += 9;
		case '+':
			servo++;
			EMIOS_0.CH[EMIOS_0_SERVO_MOTOR].CADR.R = servo;
			print("angle : ");
			i_to_s_cnt(servo, buf, 10);
			sys_log(buf);
			break;
		
		case '[':
			servo -= 9;
		case '-':
			servo--;
			EMIOS_0.CH[EMIOS_0_SERVO_MOTOR].CADR.R = servo;
			print("angle : ");
			i_to_s_cnt(servo, buf, 10);
			sys_log(buf);
			break;
		case 'O':
			set_kp(get_kp() + 100);
			goto print_kp;
		case 'o':
			set_kp(get_kp() + 10);
			goto print_kp;
		case 'l':
			set_kp(get_kp() - 10);
		case 'L':
			set_kp(get_kp() - 100);
print_kp:
			i_to_s_cnt(get_kp(), buf, 10);
			print("Kp : ");
			sys_log(buf);
			break;
		case 'i':
			set_ki(get_ki() + 1);
			goto print_ki;
		case 'I':
			set_ki(get_ki() + 10);
			goto print_ki;
		case 'K':
			set_ki(get_ki() - 10);
			goto print_ki;
		case 'k':
			set_ki(get_ki() - 1);
print_ki:
			i_to_s_cnt(get_ki(), buf, 10);
			print("Ki : ");
			sys_log(buf);
			break;
		case 'E':
			set_kd(get_kd() + 10);
			goto print_kd;
		case 'e':
			set_kd(get_kd() + 1);
			goto print_kd;
		case 'd':
			set_kd(get_kd() - 1);
			goto print_kd;
		case 'D':
			set_kd(get_kd() - 10);
			goto print_kd;
print_kd:
			i_to_s_cnt(get_kd(), buf, 10);
			print("Kd : ");
			sys_log(buf);
			break;
		case 'q':
			set_ref_speed(0);
			
			goto print_speed;
		case 'a':
			ref_speed = get_ref_speed();
			ref_speed -= 10;
			
			set_ref_speed(ref_speed);
			goto print_speed;
		case 'A':
			ref_speed = get_ref_speed();
			ref_speed -= 100;
			
			set_ref_speed(ref_speed);
			goto print_speed;
		case 'w':
			ref_speed = get_ref_speed();
			ref_speed += 10;
		
			set_ref_speed(ref_speed);
			goto print_speed;
		case 'W':
			ref_speed = get_ref_speed();
			ref_speed += 100;
		
			set_ref_speed(ref_speed);
			
			goto print_speed;
print_speed:
			i_to_s_cnt(get_ref_speed(), buf, 10);
			print("speed : ");
			sys_log(buf);
						
			break;
		case ',':{
			brake_value = brake_value +10;
			goto print_brake_value;
		}
		case '.':{
			brake_value = brake_value -10;
			goto print_brake_value;
			
		}
print_brake_value :
			i_to_s_cnt(brake_value, buf, 10);
			print("brake_value :");
			sys_log(buf);
			break;
		case '!':
			DEBUG_FUNC("kp", get_kp());
			DEBUG_FUNC("ki", get_ki());
			DEBUG_FUNC("kd", get_kd());
			DEBUG_FUNC("ref", get_ref_speed());
			DEBUG_FUNC("t_ref",brake_value);
			DEBUG_FUNC("scz_sp",SCHOOL_ZONE_SPEED_REF_LIMIT);
//			DEBUG_FUNC("theta", theta);
//			DEBUG_FUNC("top_cam", line_values_get_detected(CAMERA_TOP)[0]);
			break;
		case '1':{
			int i;
			for(i =14 ; i < 114; i++){
				DEBUG_FUNC("T_cam", line_values_get_detected(CAMERA_TOP)[i]);				
			}
			break;
		}
		case '2':{
			int i;
			for(i =14 ; i < 114; i++){
				DEBUG_FUNC("L_cam", line_values_get_detected(CAMERA_LEFT)[i]);				
			}
			break;
		}
		case '3':{
			int i;
			for(i =14 ; i < 114; i++){
				DEBUG_FUNC("R_cam", line_values_get_detected(CAMERA_RIGHT)[i]);				
			}
			break;
		}
//		case 'S':
//			ref_speed = get_ref_speed_right();
//			ref_speed += 100;
//		
//			set_ref_speed_right(ref_speed);
//			
//			goto print_speed_right;
//		case 's':
//			ref_speed = get_ref_speed_right();
//			ref_speed += 10;
//		
//			set_ref_speed_right(ref_speed);
//			
//			goto print_speed_right;
//		case 'Z':
//			ref_speed = get_ref_speed_right();
//			ref_speed -= 100;
//		
//			set_ref_speed_right(ref_speed);
//			
//			goto print_speed_right;
//		case 'z':
//			ref_speed = get_ref_speed_right();
//			ref_speed -= 10;
//		
//			set_ref_speed_right(ref_speed);
//			
//			goto print_speed_right;
//print_speed_right:
//			i_to_s_cnt(get_ref_speed_right(), buf, 10);
//			print("speed right: ");
//			sys_log(buf);
//						
//			break;
//		case 'a':
//			GPIO_SetState(69, 0);
//			break;
//		case 'b':
//			GPIO_SetState(69, 1);
//			break;
		}
	}
}

int get_draw_line_select() {
	
	return line_draw_select;
}

void set_glcd_draw_select(int glcd_draw) {
	char buf[2];

	glcd_small_clear();
	
	i_to_s_cnt(glcd_draw, buf, 2);
	drawstring(0, 1, buf);
	
	line_draw_select = glcd_draw;
}


void toggle_glcd_draw_avg() {
	draw_avg = !draw_avg;
}

void toggle_glcd_draw_mode(){
	draw_mode++;
	if ( draw_mode > 11 )
		draw_mode = 0;
}

/*
 *######################################################################
 *                           End of File
 *######################################################################
*/

