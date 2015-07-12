/*
 * line_scan.c
 *
 *  Created on: Jun 23, 2015
 *      Author: luaviskang
 */

#include "line_scan.h"
#include "adc_drv.h"
#include "st7565.h"
#include "rappid_utils.h"

#ifdef USE_CAM_1

static bool need_speed_down = false;
int avg_black = 1023;
#endif

void LineCamera_init(LineCamera * camera, pinNum serial_index_pin, pinNum clock_pin, pinNum adc_pin) {
	
	camera->adc_pin = adc_pin;
	
	camera->serial_index_pin = serial_index_pin;
	
	camera->clock_pin = clock_pin;
}

void line_scan_init() {
	
	LineCamera_init(&lineCameras[0], PIN_LINE_CAM_SI, PIN_LINE_CAM_CLOCK, PIN_LINE_CAM_1_ADC);

	LineCamera_init(&lineCameras[1], PIN_LINE_CAM_SI, PIN_LINE_CAM_CLOCK, PIN_LINE_CAM_2_ADC);

	LineCamera_init(&lineCameras[2], PIN_LINE_CAM_SI, PIN_LINE_CAM_CLOCK, PIN_LINE_CAM_3_ADC);
	
	write_pin(PIN_LINE_CAM_SI, 0);
	write_pin(PIN_LINE_CAM_CLOCK, 0);
	
#ifdef USE_CAM_1
	for(int j = 0; j < LINE_CAMERA_COUNT; j++) {
		for(int i = 0; i < LINE_CAMERA_PIXEL_CONUT; i++) { // Initialize
			line_max_min_table[j][i][CAM_MAX_VALUE_INDEX] = -1;
			line_max_min_table[j][i][CAM_MIN_VALUE_INDEX] = 1024;
		}
	}
#endif
	
}

#define MINPOINT_INTERVAL_BASE 1

#define MINIMUM_SLOPE 0

#define MAXIMIZE 1000

#define MAX_BLACK_COUNT 15

int line_calc_buf[LINE_CAMERA_PIXEL_CONUT];
int line_down_point_buf[LINE_CAMERA_PIXEL_CONUT];
int line_down_start_point_array[LINE_CAMERA_PIXEL_CONUT];

void line_calc(void) {
	
#ifndef USE_CAM_1
	
#ifdef DEBUG
	char buf[10];
#endif
	int i;
	
	int cam_index = 0;
	
	int index_interval; // index interval
	
	int start_index = INDEX_NOT_FOUND;
	int past_index = INDEX_NOT_FOUND; // past index to check last index is down point
	
	int down_point_counter = 0; // NOT USED
	
	int line_down_point_array_index = 0; // count line is down point
	
	lineValue * camera_values;
	
	// get line point values will return 
	
	int * line_down_point_array = line_down_point_buf;
	
//	// get line point delta
//	
//	int * delta_array = line_down_point_delta;
//	
	int left_last_maximum_slope = 0;
	int right_last_maximum_slope = 0;
	
	int left_last_maximum_index = INDEX_NOT_FOUND;
	int right_last_maximum_index = INDEX_NOT_FOUND;
	
	for(; cam_index < LINE_CAMERA_VALUE_LINE_COUNT; cam_index++) {
		
		// camera's adc value
		camera_values = line_values[cam_index];
		
		line_down_point_array_index = 0;
		down_point_counter = 0;
		
		start_index  = INDEX_NOT_FOUND;
		past_index = INDEX_NOT_FOUND;
		
		left_last_maximum_slope= 0;
		right_last_maximum_slope = 0;
		
		left_last_maximum_index = INDEX_NOT_FOUND;
		right_last_maximum_index = INDEX_NOT_FOUND;
		
		// check index wher current point is lower than last point
		for(i = 1; i < LINE_CAMERA_PIXEL_CONUT; i++) {
			
			if(camera_values[i] < camera_values[i - 1]) {
				
				line_calc_buf[down_point_counter] = i;
				
				down_point_counter++;
			}
		}
		
		// get minimum critical point 
		for(i = 1; i < down_point_counter; i++) {
			
			index_interval = line_calc_buf[i] -  line_calc_buf[i - 1];
			
			if(index_interval <= MINPOINT_INTERVAL_BASE) {
				
				if(start_index == INDEX_NOT_FOUND)
					start_index = line_calc_buf[i];
				
				past_index = line_calc_buf[i];
			}
			else {
				
				if(past_index != INDEX_NOT_FOUND) {
					if(past_index == LINE_CAMERA_PIXEL_CONUT / 2) { // ignore middle point
						past_index = INDEX_NOT_FOUND;
						start_index = INDEX_NOT_FOUND;
						continue;
					}
					
					// save last down point
					
					line_down_point_array[line_down_point_array_index] =  past_index;
					
					// save delta of down start and last
					
					line_down_start_point_array[line_down_point_array_index] = start_index;
					
					line_down_point_array_index++;
					
					past_index = INDEX_NOT_FOUND; // init past index
					start_index = INDEX_NOT_FOUND; // init start index
				}
			}
		}
		
		for(i = 0; i < line_down_point_array_index; i++) {
			
			if((line_down_point_array[i] - line_down_start_point_array[i]) == 0)
				continue;
			int current_slope = camera_values[line_down_start_point_array[i]] - camera_values[line_down_point_array[i]] / (line_down_point_array[i] - line_down_start_point_array[i]);
			
			if(line_down_point_array[i] < (LINE_CAMERA_PIXEL_CONUT / 2)) { // left direction index
				if(left_last_maximum_slope <  current_slope && current_slope > MINIMUM_SLOPE) {
					
					left_last_maximum_slope  = current_slope;
					
					left_last_maximum_index = line_down_point_array[i];
				}
			}
			else {
				if(right_last_maximum_slope < current_slope && current_slope > MINIMUM_SLOPE) {
					
					right_last_maximum_slope  = current_slope;
					
					right_last_maximum_index = line_down_point_array[i];
				}
			}
			
		}
		
		// critical section for wrtie value
		
//		DisableExternalInterrupts();
		
		line_point_value[cam_index][DETECTED_LEFT] = left_last_maximum_index;
		line_point_value[cam_index][DETECTED_RIGHT] = right_last_maximum_index;
//		EnableExternalInterrupts();
	}
#else
	
	int sum_point = 0;
	int sum_count = 1;
	int recent_start_black_index = INDEX_NOT_FOUND;
	int recent_stop_black_index = INDEX_NOT_FOUND;
	
#ifdef DEBUG
	char buf[10];
#endif
	
	for(int j = 0; j < LINE_CAMERA_COUNT; j++) {
		
		sum_point = 0;
		sum_count = 1;
		
		recent_start_black_index = INDEX_NOT_FOUND;
		recent_stop_black_index = INDEX_NOT_FOUND;
		
		for(int i = 0; i < LINE_CAMERA_PIXEL_CONUT; i++) {
			
			if(i < 14 || i > LINE_CAMERA_PIXEL_CONUT - 14) {
				continue;
			}
			
			if(line_values[j][i] < 
					line_max_min_table[j][i][0]) {
				sum_point += i;
				sum_count++;
			}
		}
		
		if(sum_count < 2) {
			line_point_value[j][0] = INDEX_NOT_FOUND;
		}
		else 
			line_point_value[j][0] = sum_point / sum_count;
		
		i_to_s_cnt(j, buf, 3);
		
//		print("Cam ");
//		dbg_log(buf);
//		
//		i_to_s_cnt(sum_count, buf, 10);
//		print("line count : ");
//		dbg_log(buf);
//		
//		i_to_s_cnt(line_point_value[j][0], buf, 10);
//		print("line pos : ");
//		dbg_log(buf);
//		
//		dbg_log("===========================");
		
//		if(j == 0) { // only in main camera
			if(sum_count > MAX_BLACK_COUNT) {
				need_speed_down = true;
				line_point_value[j][0] = INDEX_NOT_FOUND;
			}
			else {
				need_speed_down = false;
			}
//		}
#endif
	}
}

void line_scan() {
	
	int i = 0;
	
	pinNum general_clock = lineCameras[0].clock_pin;
	
	pinNum general_si = lineCameras[0].serial_index_pin;
	
	
	write_pin(general_si, 0);
	
	write_pin(general_clock, 1);
	write_pin(general_clock, 0);
	
	write_pin(general_clock, 1);
	write_pin(general_clock, 0);
	
	write_pin(general_si, 1);
	write_pin(general_clock, 0);

	for(i = 0; i < LINE_CAMERA_PIXEL_CONUT; i++) {
		
		write_pin(general_clock, 1);
		
		if(i < 14 || i > LINE_CAMERA_PIXEL_CONUT - 14) {
			line_values[0][i] = 1023;
			line_values[1][i] = 1023;
			line_values[2][i] = 1023;
		}
		else {
			
			line_values[0][i] = A2D_GetSingleCh_10bit(PIN_LINE_CAM_1_ADC);
			line_values[1][i] = A2D_GetSingleCh_10bit(PIN_LINE_CAM_2_ADC);
			line_values[2][i] = A2D_GetSingleCh_10bit(PIN_LINE_CAM_3_ADC);
					
		}

		if(!is_started()) {
			
			// cam 1
			
			if(line_values[0][i] > line_max_min_table[0][i][CAM_MAX_VALUE_INDEX] && 
					line_values[0][i] < MAXIMIZE
			   ) {
				
				line_max_min_table[0][i][CAM_MAX_VALUE_INDEX] = line_values[0][i];
			}
			else if(line_values[0][i] < line_max_min_table[0][i][CAM_MIN_VALUE_INDEX]) {

				line_max_min_table[0][i][CAM_MIN_VALUE_INDEX] = line_values[0][i];
			}
			
			// cam 2
			
			if(line_values[1][i] > line_max_min_table[1][i][CAM_MAX_VALUE_INDEX] && 
					line_values[1][i] < MAXIMIZE
			   ) {
				
				line_max_min_table[1][i][CAM_MAX_VALUE_INDEX] = line_values[1][i];
			}
			else if(line_values[1][i] < line_max_min_table[1][i][CAM_MIN_VALUE_INDEX]) {

				line_max_min_table[1][i][CAM_MIN_VALUE_INDEX] = line_values[1][i];
			}
			
			// cam 3
			
			if(line_values[2][i] > line_max_min_table[2][i][CAM_MAX_VALUE_INDEX] && 
					line_values[2][i] < MAXIMIZE
			   ) {
				
				line_max_min_table[2][i][CAM_MAX_VALUE_INDEX] = line_values[0][i];
			}
			else if(line_values[2][i] < line_max_min_table[2][i][CAM_MIN_VALUE_INDEX]) {

				line_max_min_table[2][i][CAM_MIN_VALUE_INDEX] = line_values[2][i];
			}
		}
		write_pin(general_clock, 0);

		write_pin(general_si, 0);		
	}
	
	write_pin(general_clock, 1);
	
	write_pin(general_clock, 0);

	
	write_pin(general_clock, 1);
	
	write_pin(general_clock, 0);	
	
	// drop 3pixel
#ifndef USE_CAM_1	
	line_values[0][0] = line_values[0][1] = line_values[0][2] = line_values[0][3];
	line_values[1][0] = line_values[1][1] = line_values[1][2] = line_values[1][3];
	
	line_values[0][127] = line_values[0][126] = line_values[0][125] = line_values[0][124];
	line_values[1][127] = line_values[1][126] = line_values[1][125] = line_values[1][124];
#endif
	
}

lineValue* line_values_get_index(int index) {

	return line_values[index];
}

int * line_values_get_detected(int index){
	
	return line_point_value[index];
}

void line_scan_draw_in_glcd(int line_num){
	
	static int selected_line_blink_animation_flag = 0;
	
	int i;
	int j;
	int y_pos = 0;
	
	lineValue * line_values = line_values_get_index(line_num);
	int * detected_line_index = line_values_get_detected(line_num);
	
	for(i = 0; i < LINE_CAMERA_PIXEL_CONUT; i++ ){

		// if detected line blink it
		
/*
		if(selected_line_blink_animation_flag < 50 && 
			(detected_line_index[0] == i ||
			detected_line_index[1] == i))
			continue;
*/
		
		y_pos = line_values[i] / 16;
		
		setpixel(i,y_pos,BLACK);
		
/*
		for(j = 63; j > y_pos; j--) {
			setpixel(i, j, BLACK);	
		}
*/
	}

/*
	selected_line_blink_animation_flag++;
	
	if(selected_line_blink_animation_flag > 99) {
		selected_line_blink_animation_flag = 0;
	}
*/
	
	// TODO For test animation work well by drawing it top of the line; remove it later
	
	if(detected_line_index[0] != INDEX_NOT_FOUND)
		setpixel(detected_line_index[0], 0, BLACK);
	
//	if(detected_line_index[1] != INDEX_NOT_FOUND)
//		setpixel(detected_line_index[1], 0, BLACK);

	glcd_display();
	
	for(i = 0; i < LINE_CAMERA_PIXEL_CONUT; i++ ){

			// if detected line blink it
			
	/*
			if(selected_line_blink_animation_flag < 50 && 
				(detected_line_index[0] == i ||
				detected_line_index[1] == i))
				continue;
	*/
			
			y_pos = line_values[i] / 16;
			
			setpixel(i,y_pos,WHITE);	
	}
	if(detected_line_index[0] != INDEX_NOT_FOUND)
			setpixel(detected_line_index[0], 0, WHITE);
		
//	if(detected_line_index[1] != INDEX_NOT_FOUND)
//		setpixel(detected_line_index[1], 0, WHITE);
	
	
}

bool is_need_to_speed_down() { // schoolzone or something
	
	return need_speed_down;
}
void make_avg_black() {
	
	int sum = 0;
	
	for(int i = 0; i < LINE_CAMERA_PIXEL_CONUT; i++) {
		if(i < 14 || i > LINE_CAMERA_PIXEL_CONUT - 14)
			continue;
		
		sum += line_max_min_table[0][i][CAM_MIN_VALUE_INDEX];
	}
	for(int i = 0; i < LINE_CAMERA_COUNT; i++) {
		for(int j = 0; j < LINE_CAMERA_PIXEL_CONUT; j++) {
			
			line_max_min_table[i][j][0] = line_max_min_table[i][j][CAM_MAX_VALUE_INDEX] - (line_max_min_table[i][j][CAM_MAX_VALUE_INDEX] - line_max_min_table[i][j][CAM_MIN_VALUE_INDEX]) * CAM_MAX_CUT_OFF / 100;
		}
	}
	
	avg_black = sum / 100;
}


int get_avg_black() {
	
	return avg_black;
}
