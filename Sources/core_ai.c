/*
 * core_ai.c
 *
 *  Created on: Jul 7, 2015
 *      Author: luaviskang
 */

#include "core_ai.h"

// common header

#include "common.h"

// sensors headers

#include "servo_motor.h"
#include "encoder.h"
#include "dc_motor.h"
#include "line_scan.h"
#include "sona_sensor.h"
#include "rappid_utils.h"
#include "school_zone_adc.h"
#include "slope.h"

long int pid_control(long int speedRf,long int feedback, bool left);
bool is_school_zon_enable = true;

#define USE_BOTTOM_CAM_ONLY

#ifdef DEBUG
#define DEBUG_FUNC(VAR_NAME, VAL) print(VAR_NAME);print(": "); i_to_s_cnt(VAL, buf, 10); sys_log(buf);
#define DEBUG_PRINT(ST) sys_log()
#else
#define DEBUG_FUNC(VAR_NAME, VAL) 
#endif

#define STRAIGHT_COUNT_LIMIT 120

#define ENCODER_ROTATE_TO_MOTOR_TORQUE(X) (X / 1000)

//#define ARC_TANGENT_TABLE_LENGTH 1431
//
//static int atan[ARC_TANGENT_TABLE_LENGTH] = {0, 1, 2, 2, 3, 3, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 15, 15, 16, 16, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 23, 24, 24, 25, 25, 26, 26, 27, 27, 28, 28, 28, 29, 29, 30, 30, 31, 31, 31, 32, 32, 33, 33, 34, 34, 34, 35, 35, 35, 36, 36, 37, 37, 37, 38, 38, 38, 39, 39, 40, 40, 40, 41, 41, 41, 42, 42, 42, 42, 43, 43, 43, 44, 44, 44, 45, 45, 45, 45, 46, 46, 46, 47, 47, 47, 47, 48, 48, 48, 48, 49, 49, 49, 49, 50, 50, 50, 50, 51, 51, 51, 51, 52, 52, 52, 52, 53, 53, 53, 53, 53, 54, 54, 54, 54, 54, 55, 55, 55, 55, 55, 56, 56, 56, 56, 56, 56, 57, 57, 57, 57, 57, 58, 58, 58, 58, 58, 58, 58, 59, 59, 59, 59, 59, 59, 60, 60, 60, 60, 60, 60, 60, 61, 61, 61, 61, 61, 61, 61, 62, 62, 62, 62, 62, 62, 62, 62, 63, 63, 63, 63, 63, 63, 63, 63, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 67, 67, 67, 67, 67, 67, 67, 67, 67, 67, 67, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 69, 69, 69, 69, 69, 69, 69, 69, 69, 69, 69, 69, 69, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86};

#define COS_CALC_VALUES_LENGTH 90

int SCHOOL_ZONE_SPEED_REF_LIMIT = 10;

// 0.5 / (cos(theta) ^ 2)

//static int cos_calc_values[COS_CALC_VALUES_LENGTH] = {1000, 989, 977, 965, 954, 942, 931, 921, 910, 899, 889, 879, 870, 860, 851, 842, 833, 825, 817, 809, 801, 794, 787, 780, 774, 768, 762, 756, 751, 746, 741, 737, 733, 729, 725, 722, 719, 717, 714, 712, 711, 709, 708, 707, 707};

static int cos_calc_values[COS_CALC_VALUES_LENGTH] = {1000, 999, 998, 996, 993, 990, 986, 981, 976, 970, 964, 957, 950, 942, 934, 925, 915, 905, 895, 884, 872, 860, 848, 835, 822, 808, 794, 780, 765, 750, 735, 720, 704, 688, 672, 655, 638, 621, 604, 587, 570, 553, 535, 518, 500, 483, 466, 448, 431, 414, 397, 380, 363, 346, 329, 313, 297, 281, 266, 250, 236, 221, 207, 193, 179, 166, 153, 141, 129, 117, 106, 96, 86, 76, 67, 59, 51, 44, 37, 31, 25, 20, 15, 11, 8, 5, 3, 2, 1, 0};
int _ref_speed = 40;
int brake_value = 30;

// TOP_CAM_DISTANCE 1.13

// MIDDLE_CAM_DISNTACE 0.65

// BODY_WIDTH 0.20

// ROAD_WIDTH 0.61

// (TOP_CAM_DISTANCE - MIDDLE_CAM_DISNTACE) / ROAD_WIDTH * 100

#ifndef USE_BOTTOM_CAM_ONLY

#define ATAN_RATIO 84
#else 
#define ATAN_RATIO 100
#endif

#define CALC_ATAN(M,W) W * ATAN_RATIO / M

#define STRAIGHT_TOLERANCE_RANGE 1 // +-2 is allow

#define SONA_CHECK_CUT_LINE 800


int sona_check_cut_line = 800;

int get_ref_speed() {

	return _ref_speed;
}

void set_ref_speed(int speed) {
	_ref_speed = speed;
}

int get_kp() {
	return kp;
}

int get_kd() {
	return kd;
}

int get_ki() {
	return ki;
}

void set_kp(int i) {
	kp = i;
}

void set_kd(int i) {
	kd = i;
}

void set_ki(int i) {
	ki = i;
}

int left_torque; // default torque
int right_torque;
int theta;
int prev_theta;

bool is_break_on = false;

void core_ai_think() {
	
	char buf[10];
	
	static int caution_not_fount_count = 0;
	static int straight_count = 0;
	static bool is_school_zone_appeared = false;
	
	int current_left_encoder_speed;
	int current_right_encoder_speed;
	
#ifndef USE_BOTTOM_CAM_ONLY
	int cam_top_center_index;
	
	int cam_top_left_index = 0;
		
	int cam_top_right_index = 129;
#endif
	
	int cam_middle_left_index = 0;
	
	int cam_middle_right_index = 129;
	
	int arc_tan_table_index = 0;
	int top_middle_point_delta = 0;
	
	int cam_middle_width = 0;
	
	bool is_left_direction = false;
	
	left_torque = 200; // default torque
	right_torque = 200;
	
	int aim_servo_motor_angle = 0;
	
#ifndef USE_BOTTOM_CAM_ONLY
	int * cam_top_values;
#endif
	
	int * cam_middle_values;

#ifndef USE_BOTTOM_CAM_ONLY	
	int * cam_bottom_values;
#endif
	
	int ref_speed = _ref_speed; // encodeer reference speed; when curve & in school zone make speed down
	
	int speed_ratio = 1000;
	
	int accel = brake_value * 1000 / _ref_speed; // rotate for 650
	
	if(_ref_speed < brake_value)  // 650 ¹®Á¦ 
		accel = 1000;
	
	theta = 0;
	
	int left_feedback = 0;
	int right_feedback = 0;
	
	int left_ref = 0;
	int right_ref = 0;
	
	bool is_need_to_stop = false; // when sonar check something
	
	bool is_found = false;
	
	bool servo_forced = false;
	
	static bool is_school_zone_entered = false;
	
	int sona_value = sona_check_cut_line;
	
#ifdef USE_CAM_1
	int line_detected_index_LEFT = INDEX_NOT_FOUND;
	int line_detected_index_RIGHT = INDEX_NOT_FOUND;
#endif

//	clear_log_screen();
	
	// read current encoder count
	
	encoder_read_left();
	encoder_read_right();
	
	// get current speed
	current_left_encoder_speed = get_left_encoder().current_delta;
	
	current_right_encoder_speed = get_right_encoder().current_delta;
	
	sona_value = sona_sensor_get().recent_distance;
	
	
	if(_abs(current_left_encoder_speed) >= 65535)
		current_left_encoder_speed = 0;
	if(_abs(current_right_encoder_speed )>= 65535)
		current_right_encoder_speed = 0;


check_current_dirct:
	
//	line_detected_index_LEFT = line_values_get_detected(CAMERA_TOP)[0];
//
//	if(line_detected_index_LEFT != INDEX_NOT_FOUND) {
//		if(line_detected_index_LEFT < 64) {
//			is_left_direction = false;
//			is_found = true;
//			theta = ( (line_detected_index_LEFT - 14) * 45 ) / 100 + 7;
//			straight_count = 0;
//		}
//		else {
//			is_left_direction = true;
//			is_found = true;
//			theta = ( (114 - line_detected_index_LEFT) * 45 ) / 100 + 7;
//			straight_count = 0;
//		}
//	}
	

	// left : turn to right

	line_detected_index_LEFT = line_values_get_detected(CAMERA_LEFT)[0];
	
	if(line_detected_index_LEFT != INDEX_NOT_FOUND) {
		
		is_left_direction = false;
		is_found = true;
		theta = ( (line_detected_index_LEFT - 14) * 45 ) / 100 + 7;
		straight_count = 0;
		
		if(theta < 10 && theta > 5) {
			accel = accel * 1200 / 1000;
		}
		else if(theta < 5) {
			accel = accel * 1300 / 1000;
		}
		
		if(accel > 1000)
			accel = 1000;
	}
	
	
	// right : turn to left
	
	line_detected_index_RIGHT = line_values_get_detected(CAMERA_RIGHT)[0];
	
	if(line_detected_index_RIGHT != INDEX_NOT_FOUND) {
		
		theta = ( (114 - line_detected_index_RIGHT) * 45) / 100 + 7;

		is_found = true;
		
		is_left_direction = true;
		
		straight_count = 0;
		
		if(theta < 8 && theta > 4) {
			accel = accel * 1200 / 1000;
		}
		else if(theta < 4) {
			accel = accel * 1300 / 1000;
		}
		
		if(accel > 1000)
			accel = 1000;
	}
	
	theta = (theta * 113) / 100; // for optimize
	
	if(_abs((is_left_direction ? -1 : 1) * theta - prev_theta) > 0 && theta > 10 && !(current_right_encoder_speed < 20 || current_left_encoder_speed < 20) && is_break_on && !is_school_zone_appeared) { // drift
		
		ref_speed = 0;
		
//		dbg_log("break!!");
	}
	
	if(is_need_to_speed_down()) {
		
		theta = 0;
		
		speed_ratio = 1000;
	}
	
	if(!is_found)
		accel = 1000;
	
	if(line_values_get_detected(CAMERA_TOP)[0] == INDEX_NOT_FOUND && !is_found) {
		
		straight_count++;
	}
	
	if(straight_count > STRAIGHT_COUNT_LIMIT) {
		
		accel = 1500;
	}
	
	if(theta >= COS_CALC_VALUES_LENGTH)
		theta = COS_CALC_VALUES_LENGTH - 1;
	
	speed_ratio = cos_calc_values[theta];

//	if(speed_ratio != 1000)
//		speed_ratio = 1000 - ((1000 - speed_ratio) * 120 / 100);

	// check school zone
	
check_school_zone:
	if ( is_school_zon_enable )
	{
		if(is_school_zone_detected() && !is_school_zone_entered) {
			
			is_school_zone_entered = true;
			
		}
		else if(!is_school_zone_detected() && is_school_zone_entered) {
			
			is_school_zone_entered = false;
			is_school_zone_appeared = !is_school_zone_appeared;
		}
		
		if(is_school_zone_appeared) {
			
			if(accel != 1000)
				accel = 1000;
			
			if(ref_speed != 0) // if not stop
				ref_speed = SCHOOL_ZONE_SPEED_REF_LIMIT;
			
			
			speed_ratio = 1000; // it is hard to curve
			
			PIT_START_TIMER_CHANNEL(PIT_CAUTION_LIGHT_CHANNEL);
		}
		else 
			PIT_STOP_TIMER_CHANNEL(PIT_CAUTION_LIGHT_CHANNEL);
	
		// get sona distance
	}	
	
check_sona:

	if(sona_value < sona_check_cut_line) {
		is_need_to_stop = true;
		ref_speed = 0;
//		dbg_log("sona stop");
	}
		
apply:	
	
//	
//	// calculate PID
//	

	if(!is_school_zone_appeared) {
		if(is_found && theta > 10) {
			if(is_left_direction) {
				write_pin(PIN_LEFT_DIR_LIGHT, 1);
				write_pin(PIN_RIGHT_DIR_LIGHT, 0);
			}
			else {
				write_pin(PIN_LEFT_DIR_LIGHT, 0);
				write_pin(PIN_RIGHT_DIR_LIGHT, 1);
			}
		}
		else if(speed_ratio >= 1000 && accel >= 1000) {
			write_pin(PIN_BREAK_LIGHT, 0);
			write_pin(PIN_RIGHT_DIR_LIGHT, 0);
			write_pin(PIN_LEFT_DIR_LIGHT, 0);
		}
		else if(ref_speed == 0 || accel < 900) {
			write_pin(PIN_BREAK_LIGHT, 1);
		}
		else {
			write_pin(PIN_BREAK_LIGHT, 0);
		}
	}
	
	left_ref = (!is_left_direction ? ref_speed * speed_ratio / 1000 : ref_speed) * accel / 1000;
	right_ref = (is_left_direction ? ref_speed : ref_speed * speed_ratio / 1000) * accel / 1000;
	
//	DEBUG_FUNC("left_ref", left_ref);
//	DEBUG_FUNC("right_ref", right_ref);
	
	left_feedback = pid_control(left_ref, current_left_encoder_speed, true);
	right_feedback = pid_control(right_ref, current_right_encoder_speed, false);

//	// apply motors
	
	left_torque = ENCODER_ROTATE_TO_MOTOR_TORQUE(left_feedback);
	
	right_torque = ENCODER_ROTATE_TO_MOTOR_TORQUE(right_feedback);
	
	// set left torque
	
	
	dc_motor_left_set_duty_rate(left_torque);
	
	// set right torque
	
	dc_motor_right_set_duty_rate(right_torque);
	
	
	// servo motor
	
	servo_motor_move((is_left_direction ? -1 : 1) * theta, servo_forced);
	prev_theta = (is_left_direction ? -1 : 1) * theta;
}

static long int left_preError[10]={0,0,0,0,0,0,0,0,0};
static long int right_preError[10]={0,0,0,0,0,0,0,0,0};

long int pid_control(long int speedRf,long int feedback, bool left) {
	
	
	int * preError = right_preError;
	
	if(left)
		preError = left_preError;
	
	
	int errorSum=0;
	
	int i;
	
	//pid controller	
//	long int kp=300;
//	long int kd=2;
//	long int ki=8;
	long int pid;
	
	long int error;
	long int errorDif;		
	
	error = speedRf - feedback; 
	errorDif = error - preError[0];
	
	errorSum = preError[9];
	//shift & sum
		
	for(i=9; i > 0; i--) {
		preError[i] = preError[i - 1];
		errorSum += preError[i];
	}
	
	//save data	
	preError[0] = error;
	errorSum +=error; 

	pid = error * kp;
	pid += errorDif * kd;
	pid += errorSum * ki;

	return pid;
}

