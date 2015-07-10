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

long int pid_control(long int speedRf,long int feedback);

static int caution_count = 0;

#ifdef DEBUG
#define DEBUG_FUNC(VAR_NAME, VAL, SIZE) print(VAR_NAME);print(": "); Uart_##SIZE(VAL)
#define DEBUG_PRINT(ST) sys_log()
#else
#define DEBUG_FUNC(VAR_NAME, VAL, SIZE) 
#endif

#define ENCODER_ROTATE_TO_MOTOR_TORQUE(X) 200 + ((X / 11) * 3 /2)

#define ARC_TANGENT_TABLE_LENGTH 262

static int atan[ARC_TANGENT_TABLE_LENGTH] = {0, 1, 2, 2, 3, 3, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 15, 15, 16, 16, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 23, 24, 24, 25, 25, 26, 26, 27, 27, 28, 28, 28, 29, 29, 30, 30, 31, 31, 31, 32, 32, 33, 33, 34, 34, 34, 35, 35, 35, 36, 36, 37, 37, 37, 38, 38, 38, 39, 39, 40, 40, 40, 41, 41, 41, 42, 42, 42, 42, 43, 43, 43, 44, 44, 44, 45, 45, 45, 45, 46, 46, 46, 47, 47, 47, 47, 48, 48, 48, 48, 49, 49, 49, 49, 50, 50, 50, 50, 51, 51, 51, 51, 52, 52, 52, 52, 53, 53, 53, 53, 53, 54, 54, 54, 54, 54, 55, 55, 55, 55, 55, 56, 56, 56, 56, 56, 56, 57, 57, 57, 57, 57, 58, 58, 58, 58, 58, 58, 58, 59, 59, 59, 59, 59, 59, 60, 60, 60, 60, 60, 60, 60, 61, 61, 61, 61, 61, 61, 61, 62, 62, 62, 62, 62, 62, 62, 62, 63, 63, 63, 63, 63, 63, 63, 63, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 67, 67, 67, 67, 67, 67, 67, 67, 67, 67, 67, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 69, 69, 69, 69, 69, 69, 69, 69, 69, 69, 69, 69, 69, 70};

#define COS_CALC_VALUES_LENGTH 90

static int cos_calc_values[COS_CALC_VALUES_LENGTH] = {815, 815, 815, 814, 814, 814, 814, 813, 813, 812, 812, 811, 811, 810, 809, 808, 807, 806, 805, 804, 803, 801, 800, 799, 797, 795, 794, 792, 790, 788, 786, 784, 781, 779, 776, 774, 771, 768, 765, 761, 758, 754, 750, 746, 742, 738, 733, 728, 723, 717, 711, 705, 699, 692, 684, 676, 668, 659, 650, 640, 629, 617, 605, 591, 577, 561, 544, 525, 504, 482, 457, 430, 399, 365, 326, 282, 232, 174, 107, 27, -70, -187, -335, -524, -777, -1132, -1664, -2550, -4325, -9648};
// TOP_CAM_DISTANCE 1.13

// MIDDLE_CAM_DISNTACE 0.65

// BODY_WIDTH 0.20

// ROAD_WIDTH 0.61

// (TOP_CAM_DISTANCE - MIDDLE_CAM_DISNTACE) / ROAD_WIDTH * 100

#define ATAN_RATIO 84

#define CALC_ATAN(M,W) W * ATAN_RATIO / M

#define STRAIGHT_TOLERANCE_RANGE 2 // +-2 is allow

void core_ai_think() {
	
	int current_left_encoder_speed;
	int current_right_encoder_speed;
	
	int cam_top_center_index;
	int cam_middle_center_index;
	
	int arc_tan_table_index = 0;
	
	int top_middle_point_delta = 0;
	
	int cam_middle_width = 0;
	
	bool is_left_direction = false;
	
	int aim_servo_motor_angle = 0;
	
	int * cam_top_values;
	
	int * cam_middle_values;
	
	int * cam_bottom_values;
	
	int ref_speed = 4000; // encodeer reference speed; when curve & in school zone make speed down
	
	int speed_ratio = 1;
	
	int theta = 90;
	
	int corner_theta = 90;
	
	int left_feedback = 0;
	int right_feedback = 0;
	
	int left_torque = 200; // default torque
	int right_torque = 200;
	
	// get current speed
	current_left_encoder_speed = get_left_encoder().current_delta;
//	
	current_right_encoder_speed = get_right_encoder().current_delta;
	
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
	
	cam_top_center_index = (cam_top_values[DETECTED_RIGHT] - cam_top_values[DETECTED_LEFT]) / 2;
	cam_middle_center_index = (cam_middle_values[DETECTED_RIGHT] - cam_middle_values[DETECTED_LEFT]) / 2;
	
	cam_middle_width = cam_middle_values[DETECTED_RIGHT] - cam_middle_values[DETECTED_LEFT];
	cam_middle_width = cam_middle_width == 0 ? 1 : cam_middle_width; // TODO Change it plz, would you?
	
	if(cam_top_center_index < 0 || cam_middle_center_index < 0) { // it never be happened, but if this situation occurred it is dangerous 
		
		danger_level = DangerLevelRescue;
	}
	else {
		top_middle_point_delta = (cam_top_center_index - cam_middle_center_index);

		DEBUG_FUNC("top middle delta", top_middle_point_delta, 100);
		
		if(top_middle_point_delta < 0) { // if middle is bigger than top, it said need to turn left
			top_middle_point_delta *= -1;
			is_left_direction = true;
		}
		
		if(top_middle_point_delta != STRAIGHT_TOLERANCE_RANGE) { // calculate curve
			
			arc_tan_table_index = CALC_ATAN(top_middle_point_delta, cam_middle_width);
			
			if(arc_tan_table_index >= ARC_TANGENT_TABLE_LENGTH) {
				arc_tan_table_index = ARC_TANGENT_TABLE_LENGTH - 1;
			}
			
			theta = atan[arc_tan_table_index];
			
			DEBUG_FUNC("theta", theta, 100);
			
			corner_theta = 180 - 2 * theta;
			
			if(corner_theta > COS_CALC_VALUES_LENGTH) { // it will become danger
				corner_theta = 0;
				danger_level = DangerLevelRescue;
			}
			else {
				speed_ratio = cos_calc_values[corner_theta];
				DEBUG_FUNC("speed_ratio", speed_ratio, 100);
			}
		}
		else { // if top middle point delta is +-2 it is almost straight way
			speed_ratio = 1;
			dbg_log("straight");
		}
	}
		
	// check it is slope

	// check school zone
	
	// get sona distance

	// calculate PID
	
	left_feedback = pid_control(current_left_encoder_speed, (is_left_direction ? ref_speed * speed_ratio : ref_speed));
	right_feedback = pid_control(current_left_encoder_speed, (is_left_direction ? ref_speed : ref_speed * speed_ratio));
	
	// apply motors
	
	left_torque = ENCODER_ROTATE_TO_MOTOR_TORQUE(left_feedback);
	
	right_torque = ENCODER_ROTATE_TO_MOTOR_TORQUE(right_feedback);
	
	// left torque
	
	dc_motor_left_set_duty_rate(left_torque);
	
	//right torque
	
	dc_motor_right_set_duty_rate(right_torque);
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
	pid+= errorDif * kd;
	pid+= errorSum * ki;
	
	//shift
	for(i=9;i>0;i--)
		preError[i] = preError[i-1];	
	//save data	
	preError[0] = error;

	return pid;
}

