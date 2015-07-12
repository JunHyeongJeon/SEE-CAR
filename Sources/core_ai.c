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

#include "encoder.h"
#include "dc_motor.h"
#include "line_scan.h"
#include "sona_sensor.h"

static int caution_count = 0;

#define ARC_TANGENT_TABLE_LENGTH 262
static int atan[ARC_TANGENT_TABLE_LENGTH] = {0, 1, 2, 2, 3, 3, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 15, 15, 16, 16, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 23, 24, 24, 25, 25, 26, 26, 27, 27, 28, 28, 28, 29, 29, 30, 30, 31, 31, 31, 32, 32, 33, 33, 34, 34, 34, 35, 35, 35, 36, 36, 37, 37, 37, 38, 38, 38, 39, 39, 40, 40, 40, 41, 41, 41, 42, 42, 42, 42, 43, 43, 43, 44, 44, 44, 45, 45, 45, 45, 46, 46, 46, 47, 47, 47, 47, 48, 48, 48, 48, 49, 49, 49, 49, 50, 50, 50, 50, 51, 51, 51, 51, 52, 52, 52, 52, 53, 53, 53, 53, 53, 54, 54, 54, 54, 54, 55, 55, 55, 55, 55, 56, 56, 56, 56, 56, 56, 57, 57, 57, 57, 57, 58, 58, 58, 58, 58, 58, 58, 59, 59, 59, 59, 59, 59, 60, 60, 60, 60, 60, 60, 60, 61, 61, 61, 61, 61, 61, 61, 62, 62, 62, 62, 62, 62, 62, 62, 63, 63, 63, 63, 63, 63, 63, 63, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 67, 67, 67, 67, 67, 67, 67, 67, 67, 67, 67, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 69, 69, 69, 69, 69, 69, 69, 69, 69, 69, 69, 69, 69, 70};

#define CAM_DISTANCE_TO_CAM_WIDTH_RATIO	214

void core_ai_think() {
	
	int current_left_encoder_speed;
	int current_right_encoder_speed;
	
	int cam_top_center_index;
	int cam_middle_center_index;
	
	int arc_tan_table_index = 0;
	int top_middle_point_delta = 0;
	
	bool is_left_direction = false;
	
	
	int aim_servo_motor_angle = 0;
	
	int * cam_top_values;
	
	int * cam_middle_values;
	
	int * cam_bottom_values;
	
	// get current speed
	current_left_encoder_speed = get_left_encoder().current_delta;
//	
	current_right_encoder_speed = get_right_encoder().current_delta;
	
	// check school zone
	
	// get camera values
	
	cam_top_values = line_values_get_detected(CAMERA_TOP);
	
	cam_middle_values = line_values_get_detected(CAMERA_MIDDLE);
	
	// check is mode could be dangerous
	
	if(cam_top_values[DETECTED_LEFT] == INDEX_NOT_FOUND ||
	   cam_top_values[DETECTED_RIGHT] == INDEX_NOT_FOUND ||
	   cam_middle_values[DETECTED_LEFT] == INDEX_NOT_FOUND ||
	   cam_middle_values[DETECTED_RIGHT] == INDEX_NOT_FOUND
	   ) {
		caution_count++; // count not found
	}
	else {
		caution_count = 0;
		danger_level = DangerLevelSafeMode;
	}
	
	if(caution_count > COUNT_NOT_FOUND_LIMIT) {
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
	
	// get curve degree
	
	/*
	 * ! note
	 * 
	 * 		if detect left line failed, it just use -1.
	 * 		So there is no reason to convert value like right not found case
	 * 		
	 */
	
	if(cam_top_values[DETECTED_RIGHT] == INDEX_NOT_FOUND)
		cam_top_values[DETECTED_RIGHT] = LINE_CAMERA_PIXEL_CONUT + 1;
	
	if(cam_middle_values[DETECTED_RIGHT] == INDEX_NOT_FOUND)
			cam_middle_values[DETECTED_RIGHT] = LINE_CAMERA_PIXEL_CONUT + 1;
	
	// get center index
	
	cam_top_center_index = cam_top_values[DETECTED_RIGHT] - cam_top_values[DETECTED_LEFT];
	cam_middle_center_index = cam_middle_values[DETECTED_RIGHT] - cam_middle_values[DETECTED_LEFT];
	
	if(cam_top_center_index < 0 || cam_middle_center_index < 0) { // it never be happened, but if this situation occurred it is dangerous 
		
		danger_level = DangerLevelRescue;
	}
	else {
		top_middle_point_delta = (cam_top_center_index - cam_middle_center_index);
		
		if(top_middle_point_delta < 0) { // if middle is bigger than top, it said need to turn left
			top_middle_point_delta *= -1;
			is_left_direction = true;
		}
		
		arc_tan_table_index = top_middle_point_delta * 100 / CAM_DISTANCE_TO_CAM_WIDTH_RATIO;
		
		arc_tan_table_index /= 100; // divide 100 because ratio is already multiple 100
		
		if(arc_tan_table_index >= ARC_TANGENT_TABLE_LENGTH) {
			arc_tan_table_index = ARC_TANGENT_TABLE_LENGTH - 1;
		}
		
		
		aim_servo_motor_angle = atan[arc_tan_table_index];
	}
	
	// get sona distance
	
	// check it is slope
	
	// apply motors
	
	if(aim_servo_motor_angle < 20) {
		
	}
	
}
