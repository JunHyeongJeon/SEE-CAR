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
	for(int i = 0; i < LINE_CAMERA_PIXEL_CONUT; i++) { // Initialize
		line_max_min_table[i][CAM_MAX_VALUE_INDEX] = -1;
		line_max_min_table[i][CAM_MIN_VALUE_INDEX] = 1024;
	}
#endif
	
}

#define MINPOINT_INTERVAL_BASE 1

#define MINIMUM_SLOPE 0

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
	int sum_count = 0;
	
	for(int i = 0; i < LINE_CAMERA_PIXEL_CONUT; i++) {
		if(i > 14 || i < LINE_CAMERA_VALUE_LINE_COUNT - 14) {
			continue;
		}
		
		if(line_values[CAMERA_TOP][i] > 
				(line_max_min_table[i][CAM_MAX_VALUE_INDEX] - line_max_min_table[i][CAM_MIN_VALUE_INDEX]) * 100 / CAM_MAX_CUT_OFF
				+ line_max_min_table[i][CAM_MIN_VALUE_INDEX]) {
			
			sum_point += i;
			sum_count++;
		}
	}
	
	if(sum_count == 0 || sum_count == 1) {
		line_point_value[CAMERA_TOP][0] = INDEX_NOT_FOUND;
	}
	
	line_point_value[CAMERA_TOP][0] = sum_point / sum_count;
#endif
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
		
		line_values[0][i] = A2D_GetSingleCh_10bit(PIN_LINE_CAM_1_ADC);
		
		if(i < 14 || i > LINE_CAMERA_PIXEL_CONUT - 14) {
			line_values[0][i] = 1023;
		}
		
#ifndef USE_CAM_1
		if(i % 2 ==0) {
			line_values[1][i / 2] = A2D_GetSingleCh_10bit(PIN_LINE_CAM_3_ADC);
			line_values[1][64 + i / 2] = A2D_GetSingleCh_10bit(PIN_LINE_CAM_2_ADC);
		}
		else {
			line_values[1][i / 2] = (A2D_GetSingleCh_10bit(PIN_LINE_CAM_3_ADC) + line_values[1][i / 2]) / 2;
			line_values[1][64 + i / 2] = (A2D_GetSingleCh_10bit(PIN_LINE_CAM_2_ADC) + line_values[1][64 + i / 2]) / 2;
		}
#else
		if(!is_started()) {
			if(line_values[0][i] > line_max_min_table[i][CAM_MAX_VALUE_INDEX]) {
				
				line_max_min_table[i][CAM_MAX_VALUE_INDEX] = line_values[0][i];
			}
			else if(line_values[0][i] < line_max_min_table[i][CAM_MIN_VALUE_INDEX]) {

				line_max_min_table[i][CAM_MIN_VALUE_INDEX] = line_values[0][i];
			}
		}
#endif
		
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

#ifdef USE_CAM_1
lineValue * line_values_get_max_min(int index) {

	return line_max_min_table[index];
}
#endif

void line_scan_draw_in_glcd(int line_num){
	
	static int selected_line_blink_animation_flag = 0;
	
	int i;
	int j;
	int y_pos = 0;
	
	lineValue * line_values = line_values_get_index(line_num);
	int * detected_line_index = line_values_get_detected(line_num);
	
	for(i = 0; i < LINE_CAMERA_PIXEL_CONUT; i++ ){

		// if detected line blink it
		
		if(selected_line_blink_animation_flag < 50 && 
			(detected_line_index[0] == i ||
			detected_line_index[1] == i))
			continue;
		
		y_pos = line_values[i] / 16;
		
		for(j = 63; j > y_pos; j--) {
			setpixel(i, j, BLACK);	
		}
	}
	
	selected_line_blink_animation_flag++;
	
	if(selected_line_blink_animation_flag > 99) {
		selected_line_blink_animation_flag = 0;
	}
	
	// TODO For test animation work well by drawing it top of the line; remove it later
	
	if(detected_line_index[0] != INDEX_NOT_FOUND)
		setpixel(detected_line_index[0], 0, BLACK);
	
	if(detected_line_index[1] != INDEX_NOT_FOUND)
		setpixel(detected_line_index[1], 0, BLACK);
	
}

