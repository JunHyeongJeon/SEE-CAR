/*
 * line_scan.c
 *
 *  Created on: Jun 23, 2015
 *      Author: luaviskang
 */

#include "line_scan.h"
#include "adc_drv.h"


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
}

#define MINPOINT_INTERVAL_BASE 1

#define INDEX_DELTA_MINIMUM 2 // for thickness of line

int line_calc_buf[LINE_CAMERA_PIXEL_CONUT];
int line_down_point_buf[LINE_CAMERA_PIXEL_CONUT];
int line_down_point_delta[LINE_CAMERA_PIXEL_CONUT];

void line_calc(void) {
	
	int i;
	
	int cam_index = 0;
	
	int index_interval; // index interval
	
	int start_index = INDEX_NOT_FOUND;
	int past_index = INDEX_NOT_FOUND; // past index to check last index is down point
	
	int down_point_counter = 0; // NOT USED
	
	int line_down_point_array_index = 0; // count line is down point
	
	lineValue * camera_values;
	
	// get line point values will return 
	
	lineValue * line_down_point_array = line_down_point_buf;
	
	// get line point delta
	
	lineValue * delta_array = line_down_point_delta;
	
	lineValue left_last_maximum_delta = 0;
	lineValue right_last_maximum_delta = 0;
	
	int left_last_maximum_index = INDEX_NOT_FOUND;
	int right_last_maximum_index = INDEX_NOT_FOUND;
	
	for(; cam_index < LINE_CAMERA_COUNT - 1; cam_index++) {
		
		// camera's adc value
		camera_values = line_values[cam_index];
		
		line_down_point_array_index = 0;
		down_point_counter = 0;
		
		start_index  = INDEX_NOT_FOUND;
		past_index = INDEX_NOT_FOUND;
		
		left_last_maximum_delta = 0;
		right_last_maximum_delta = 0;
		
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
					
					delta_array[line_down_point_array_index] = past_index - start_index;
					
					line_down_point_array_index++;
					
					past_index = INDEX_NOT_FOUND; // init past index
					start_index = INDEX_NOT_FOUND; // init start index
				}
			}
		}
		
		for(i = 0; i < line_down_point_array_index; i++) {
			
			if(line_down_point_array[i] < (LINE_CAMERA_PIXEL_CONUT / 2)) { // left direction index
				if(left_last_maximum_delta < delta_array[i] && delta_array[i] > INDEX_DELTA_MINIMUM) {
					
					left_last_maximum_delta  = delta_array[i];
					
					left_last_maximum_index = line_down_point_array[i];
				}
			}
			else {
				if(right_last_maximum_delta < delta_array[i] && delta_array[i] > INDEX_DELTA_MINIMUM) {
					
					right_last_maximum_delta  = delta_array[i];
					
					right_last_maximum_index = line_down_point_array[i];
				}
			}
			
		}
		
		line_point_value[cam_index][DETECTED_LEFT] = left_last_maximum_index;
		line_point_value[cam_index][DETECTED_RIGHT] = right_last_maximum_index;
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
		
		line_values[0][i] = A2D_GetSingleCh_10bit(PIN_LINE_CAM_1_ADC);
		
		if(i % 2 ==0) {
			line_values[1][i / 2] = A2D_GetSingleCh_10bit(PIN_LINE_CAM_3_ADC);
			line_values[1][64 + i / 2] = A2D_GetSingleCh_10bit(PIN_LINE_CAM_2_ADC);
		}
		else {
			line_values[1][i / 2] = (A2D_GetSingleCh_10bit(PIN_LINE_CAM_3_ADC) + line_values[1][i / 2]) / 2;
			line_values[1][64 + i / 2] = (A2D_GetSingleCh_10bit(PIN_LINE_CAM_2_ADC) + line_values[1][64 + i / 2]) / 2;
		}
		
		write_pin(general_clock, 0);

		write_pin(general_si, 0);		
	}
	
	write_pin(general_clock, 1);
	
	write_pin(general_clock, 0);

	
	write_pin(general_clock, 1);
	
	write_pin(general_clock, 0);	
	
	
	
}

lineValue* line_values_get_index(int index) {

	return line_values[index];
}

int * line_values_get_detected(int index){
	
	return line_point_value[index];
}
