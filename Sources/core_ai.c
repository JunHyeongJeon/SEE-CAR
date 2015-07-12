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

long int pid_control(long int speedRf,long int feedback);

#define USE_BOTTOM_CAM_ONLY

//#ifdef DEBUG
//#define DEBUG_FUNC(VAR_NAME, VAL) print(VAR_NAME);print(": "); i_to_s_cnt(VAL, buf, 10); sys_log(buf);
//#define DEBUG_PRINT(ST) sys_log()
//#else
#define DEBUG_FUNC(VAR_NAME, VAL) 
//#endif

#define ENCODER_ROTATE_TO_MOTOR_TORQUE(X) (X == 0 ? 0 : ((X < 0 ? -200 : 200) + ((X / 1000))))

#define ARC_TANGENT_TABLE_LENGTH 1431

static int atan[ARC_TANGENT_TABLE_LENGTH] = {0, 1, 2, 2, 3, 3, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 15, 15, 16, 16, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 23, 24, 24, 25, 25, 26, 26, 27, 27, 28, 28, 28, 29, 29, 30, 30, 31, 31, 31, 32, 32, 33, 33, 34, 34, 34, 35, 35, 35, 36, 36, 37, 37, 37, 38, 38, 38, 39, 39, 40, 40, 40, 41, 41, 41, 42, 42, 42, 42, 43, 43, 43, 44, 44, 44, 45, 45, 45, 45, 46, 46, 46, 47, 47, 47, 47, 48, 48, 48, 48, 49, 49, 49, 49, 50, 50, 50, 50, 51, 51, 51, 51, 52, 52, 52, 52, 53, 53, 53, 53, 53, 54, 54, 54, 54, 54, 55, 55, 55, 55, 55, 56, 56, 56, 56, 56, 56, 57, 57, 57, 57, 57, 58, 58, 58, 58, 58, 58, 58, 59, 59, 59, 59, 59, 59, 60, 60, 60, 60, 60, 60, 60, 61, 61, 61, 61, 61, 61, 61, 62, 62, 62, 62, 62, 62, 62, 62, 63, 63, 63, 63, 63, 63, 63, 63, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 67, 67, 67, 67, 67, 67, 67, 67, 67, 67, 67, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 69, 69, 69, 69, 69, 69, 69, 69, 69, 69, 69, 69, 69, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 71, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86};

#define COS_CALC_VALUES_LENGTH 90

// 0.5 / (cos(theta) ^ 2)

static int cos_calc_values[COS_CALC_VALUES_LENGTH] = {0, 1, 2, 3, 5, 8, 11, 15, 20, 25, 31, 37, 44, 51, 59, 67, 76, 86, 96, 106, 117, 129, 141, 153, 166, 179, 193, 207, 221, 236, 250, 266, 281, 297, 313, 329, 346, 363, 380, 397, 414, 431, 448, 466, 483, 500, 518, 535, 553, 570, 587, 604, 621, 638, 655, 672, 688, 704, 720, 735, 750, 765, 780, 794, 808, 822, 835, 848, 860, 872, 884, 895, 905, 915, 925, 934, 942, 950, 957, 964, 970, 976, 981, 986, 990, 993, 996, 998, 999, 1000};
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

void core_ai_think() {
	
	char buf[10];
	
	static int caution_not_fount_count = 0;

	
	int current_left_encoder_speed;
	int current_right_encoder_speed;
	
#ifndef USE_BOTTOM_CAM_ONLY
	int cam_top_center_index;
	
	int cam_top_left_index = 0;
		
	int cam_top_right_index = 129;
#endif
	int cam_middle_center_index;
	
	int cam_middle_left_index = 0;
	
	int cam_middle_right_index = 129;
	
	int arc_tan_table_index = 0;
	
	int top_middle_point_delta = 0;
	
	int cam_middle_width = 0;
	
	bool is_left_direction = false;
	
	int aim_servo_motor_angle = 0;
	
#ifndef USE_BOTTOM_CAM_ONLY
	int * cam_top_values;
#endif
	
	int * cam_middle_values;

#ifndef USE_BOTTOM_CAM_ONLY	
	int * cam_bottom_values;
#endif
	
	int ref_speed = 1600; // encodeer reference speed; when curve & in school zone make speed down
	
	int speed_ratio = 1000;
	
	int theta = 90;
	
	int left_feedback = 0;
	int right_feedback = 0;
	
	int left_torque = 200; // default torque
	int right_torque = 200;
	
	bool is_need_to_stop = false; // when sonar check something
	
	int sona_value = SONA_CHECK_CUT_LINE;
	
#ifdef USE_CAM_1
	int line_detected_index = INDEX_NOT_FOUND;
#endif
	
#ifdef DEBUG
	// for get clean status
	clear_log_screen();
#endif
	
	
// critical section to read value
//	DisableExternalInterrupts();
	

#ifndef USE_CAM_1
	
	// get camera values
	#ifndef USE_BOTTOM_CAM_ONLY	
	cam_top_values = line_values_get_detected(CAMERA_TOP);
	#endif
	cam_middle_values = line_values_get_detected(CAMERA_MIDDLE);
#endif
	// read current encoder count
	
	encoder_read_left();
	encoder_read_right();
	
	// get current speed
	current_left_encoder_speed = get_left_encoder().current_delta;
//	
	current_right_encoder_speed = get_right_encoder().current_delta;
	
#ifndef USE_CAM_1
	
#ifndef USE_BOTTOM_CAM_ONLY
	cam_top_left_index = cam_middle_values[DETECTED_LEFT];
			
	cam_top_right_index = cam_middle_values[DETECTED_RIGHT];
#endif
	
	cam_middle_right_index = cam_middle_values[DETECTED_RIGHT];
	cam_middle_left_index = cam_middle_values[DETECTED_LEFT];
#endif
	
	sona_value = sona_sensor_get().recent_distance;
	
//	EnableExternalInterrupts();
	
	if(current_left_encoder_speed >= 65535)
		current_left_encoder_speed = 0;
	if(current_right_encoder_speed >= 65535)
		current_right_encoder_speed = 0;
	
	DEBUG_FUNC("current_left_encoder_speed", current_left_encoder_speed);
	DEBUG_FUNC("current_right_encoder_speed", current_right_encoder_speed);	
	
	// check is mode could be dangerous
#ifndef USE_CAM_1
		
	if(
#ifndef USE_BOTTOM_CAM_ONLY
		cam_top_left_index == INDEX_NOT_FOUND ||
		cam_top_right_index == INDEX_NOT_FOUND ||
#endif
	   cam_middle_left_index == INDEX_NOT_FOUND ||
	   cam_middle_right_index == INDEX_NOT_FOUND
	   ) {
		if(caution_not_fount_count < COUNT_NOT_FOUND_LIMIT)
			caution_not_fount_count++; // count not found
	}
	else {
		caution_not_fount_count = 0;
		danger_level = DangerLevelSafeMode;
	}

	if(caution_not_fount_count > COUNT_NOT_FOUND_LIMIT) {
		switch (danger_level){
		case DangerLevelSafeMode:
			danger_level = DangerLevelCaution;
			break;
		case DangerLevelCaution:
			danger_level = DangerLevelRescue;
			break;
		default: // It really dangerous..
			danger_level = DangerLevelRescue;
		}
	}
#endif
	
	// get curve degree
	
	/*
	 * ! note
	 * 
	 * 		if detect left line failed, it just use -1.
	 * 		So there is no reason to convert value like right not found case
	 * 		
	 */
#ifndef USE_CAM_1
#ifndef USE_BOTTOM_CAM_ONLY
	if(cam_top_right_index == INDEX_NOT_FOUND)
		cam_top_right_index = LINE_CAMERA_PIXEL_CONUT + 1;
#endif
	if(cam_middle_right_index == INDEX_NOT_FOUND)
		cam_middle_right_index = LINE_CAMERA_PIXEL_CONUT + 1;
	
	// get center index
#ifndef USE_BOTTOM_CAM_ONLY
	DEBUG_FUNC("cam_top_values[DETECTED_RIGHT]", cam_top_left_index);
	DEBUG_FUNC("cam_top_values[DETECTED_LEFT]", cam_top_right_index);
	
	cam_top_center_index = (cam_top_right_index - cam_top_left_index) / 2;
#endif
	
	DEBUG_FUNC("cam_middle_values[DETECTED_RIGHT]", cam_middle_right_index);
	DEBUG_FUNC("cam_middle_values[DETECTED_LEFT]", cam_middle_left_index);
	
	cam_middle_width = cam_middle_right_index - cam_middle_left_index;
	cam_middle_width = cam_middle_width == 0 ? 1 : cam_middle_width; // TODO Change it plz, would you?	
	
	cam_middle_center_index = cam_middle_width / 2;
	
	DEBUG_FUNC("cam_middle_center_index", cam_middle_center_index);
	DEBUG_FUNC("cam_middle_width", cam_middle_width);
	
	if(
#ifndef USE_BOTTOM_CAM_ONLY
		cam_top_center_index < 0 ||
#endif
		cam_middle_center_index < 0) { // it never be happened, but if this situation occurred it is dangerous 
		
		danger_level = DangerLevelRescue;
		return;
	}
	else {
		
#ifndef USE_BOTTOM_CAM_ONLY
		top_middle_point_delta = (cam_top_center_index - cam_middle_center_index);
#else
		top_middle_point_delta = cam_middle_center_index - (40);
#endif		
		
		if(top_middle_point_delta < 0) { // if middle is bigger than top, it said need to turn left
			
			top_middle_point_delta = top_middle_point_delta * -1;
			is_left_direction = true;
		}
		else {
			is_left_direction = false;
		}
		
		DEBUG_FUNC("top middle delta", top_middle_point_delta);
		
#ifdef DEBUG
		if(!is_left_direction) {
			dbg_log("Right direction");
		}
		else {
			dbg_log("Left direction");
		}
#endif
		
		dbg_log("check top middle point delta");
		
		if(top_middle_point_delta != STRAIGHT_TOLERANCE_RANGE) { // calculate curve
			
			arc_tan_table_index = CALC_ATAN(top_middle_point_delta, cam_middle_width);
			
			DEBUG_FUNC("arc_tan_table_index", arc_tan_table_index);
			
			if(arc_tan_table_index < 0) {
				return; // It must not happen; :)
			}
			
			if(arc_tan_table_index >= ARC_TANGENT_TABLE_LENGTH) { // it mean almost 90'd
				
				arc_tan_table_index = ARC_TANGENT_TABLE_LENGTH - 1;
				theta = 90;
				speed_ratio = 1000;
				dbg_log("straight");
			}
			else {
				
				theta = atan[arc_tan_table_index];
				
				DEBUG_FUNC("theta", theta);
				
				if(theta > COS_CALC_VALUES_LENGTH) { // it will become danger
					theta = 0;
					danger_level = DangerLevelRescue;
				}
				else {
					
					// 0.105 * 1000 => body half size
					
					speed_ratio = ((cos_calc_values[theta] * top_middle_point_delta - 105) * 1000) / (cos_calc_values[theta] * top_middle_point_delta + 105);
					
					DEBUG_FUNC("speed_ratio", speed_ratio);
				}
			}
		}
		else { // if top middle point delta is +-2 it is almost straight way
			speed_ratio = 1000;
			theta = 90;
			dbg_log("straight");
		}
	}
#else
	
	line_detected_index = line_values_get_detected(CAMERA_TOP)[0];
	
	if(line_detected_index == INDEX_NOT_FOUND) {
		theta = 89;
		speed_ratio = 1000;
	}
	else {
		if(line_detected_index < LINE_CAMERA_PIXEL_HALF_COUNT) {
			
			theta = 90 - ((line_detected_index - 14) * 90 / LINE_CAMERA_PIXEL_HALF_COUNT);
			
			is_left_direction = false;
		}
		else {
			
			theta = 90 - ((100 - line_detected_index) * 90 / LINE_CAMERA_PIXEL_HALF_COUNT);
			
			is_left_direction = true;
			
		}
		
		theta = theta * 117 / 100;
		
		DEBUG_FUNC("line_detected_index ", line_detected_index);
		DEBUG_FUNC("theta", theta);
	}
	
	if(is_need_to_speed_down()) {
//		theta = 90;
//		speed_ratio = 1000;
		theta = servo_get_current_angle();
		if(theta < 0) {
			theta = -1 * theta;
			is_left_direction = true;
		}
		else {
			is_left_direction = false;
		}
		
		theta = 90 - theta; // get last value
		
		dbg_log("School zone detected");
		return;
	}
	
	if(line_values_get_detected(CAMERA_LEFT)[0] != INDEX_NOT_FOUND) {
		
		theta += (line_values_get_detected(CAMERA_LEFT)[0] - 14) * 45 / 100;
	}
	if(line_values_get_detected(CAMERA_RIGHT)[0] != INDEX_NOT_FOUND) {
		
		theta -= (100 - line_values_get_detected(CAMERA_RIGHT)[0]) * 45 / 100;
	}
//	else { // if not dangerous; do not too bigger turning
		
//		int temp_theta = (is_left_direction ? -1 * theta : theta);
//		
//		if(_abs(temp_theta - (90 + servo_get_current_angle())) > 50) {
//			theta = (90 + servo_get_current_angle());
//			
//			if(theta < 0) {
//				
//				theta = -1 * theta;
//				
//				is_left_direction = true;
//			}
//			else {
//				
//				is_left_direction = false;
//			}
//		}
//	}
	
	
	if(theta >= COS_CALC_VALUES_LENGTH)
		theta = COS_CALC_VALUES_LENGTH - 1;
	
	speed_ratio = cos_calc_values[theta];
	speed_ratio = speed_ratio - (1000 - speed_ratio) / 20; // for optimize
#endif
check_slope:
	
	// check it is slope

	// check school zone
	
	// get sona distance
	
check_sona:

//	if(sona_value < SONA_CHECK_CUT_LINE) {
//		is_need_to_stop = true;
//		ref_speed = 0;
//	}
	
	// check mode is caution or rescue
	
check_mode:
	switch(danger_level) {
	case DangerLevelCaution:
		ref_speed = ref_speed * 3 / 4; // in caution mode 75% speed down
		break;
	case DangerLevelRescue:
		ref_speed = ref_speed / 2; // in rescue mode 50% speed down to find line
		break;
	default :
		break;
	}
	
#ifdef DEBUG // in debug mode slow down
	
	if(danger_level == DangerLevelCaution)
		dbg_log("In cautioin mode");
	else if(danger_level == DangerLevelRescue)
		dbg_log("In danger mode");
	
	ref_speed = ref_speed * 3 / 4;
#endif 
	
	DEBUG_FUNC("reference speed", ref_speed);
	DEBUG_FUNC("speed_ratio", speed_ratio);
	
	// calculate PID
	
	left_feedback = pid_control((is_left_direction ? ref_speed * speed_ratio / 1000 : ref_speed), current_left_encoder_speed);
	right_feedback = pid_control((!is_left_direction ? ref_speed : ref_speed * speed_ratio / 1000), current_right_encoder_speed);
	
	DEBUG_FUNC("left_feedback", left_feedback);
	DEBUG_FUNC("right_feedback", right_feedback);
	
	// apply motors
	
	left_torque = ENCODER_ROTATE_TO_MOTOR_TORQUE(left_feedback);
	
	right_torque = ENCODER_ROTATE_TO_MOTOR_TORQUE(right_feedback);
	
	DEBUG_FUNC("left_torque", left_torque);
	DEBUG_FUNC("right_torque", right_torque);	
	
	// set left torque
	
	dc_motor_left_set_duty_rate(left_torque);
	
//	// set right torque
	
	dc_motor_right_set_duty_rate(right_torque);
	
	
	// servo motor
	
	servo_motor_move((is_left_direction ? -1 : 1) * (90 - theta));
}

long int pid_control(long int speedRf,long int feedback) {
	
	static long int preError[10]={0,0,0,0,0,0,0,0,0};
	static long int errorSum=0;
	volatile i;
	//pid controller	
	long int kp=300;
	long int kd=2;
	long int ki=8;
	long int pid;
	
	long int error;
	long int errorDif;		
	
	error = speedRf - feedback; 
	errorDif = error - preError[0];
	
	errorSum = 0;
	for(i=0;i<9;i++)
		errorSum +=preError[i];
	
	errorSum +=error; 

	pid = error * kp;
	pid += errorDif * kd;
	pid += errorSum * ki;
	
	//shift
	for(i=9;i>0;i--)
		preError[i] = preError[i-1];	
	//save data	
	preError[0] = error;

	return pid;
}

