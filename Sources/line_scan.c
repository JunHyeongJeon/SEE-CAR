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
#include "sona_sensor.h"
#include "core_ai.h"


extern int sona_check_cut_line;
extern bool is_school_zon_enable;
extern bool is_glcd_enable;
extern DRAW_MODE draw_mode;
lineValue line_max_min_table[LINE_CAMERA_COUNT][LINE_CAMERA_PIXEL_CONUT][2];

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

	
	int sum_point = 0;
	int sum_count = 1;
	int recent_start_black_index = INDEX_NOT_FOUND;
	int recent_stop_black_index = INDEX_NOT_FOUND;
	
#ifdef DEBUG
	char buf[10];
#endif
	
	int schoolzone_count = 0;
	
	for(int j = 0; j < LINE_CAMERA_COUNT; j++) {
		
		sum_point = 0;
		sum_count = 0;
		
		recent_start_black_index = INDEX_NOT_FOUND;
		recent_stop_black_index = INDEX_NOT_FOUND;
		
		for(int i = 0; i < LINE_CAMERA_PIXEL_CONUT; i++) {
			
			if(i < 14 || i > LINE_CAMERA_PIXEL_CONUT - 14) {
				continue;
			}
			
			if(line_values[j][i] < 
					line_max_min_table[j][i][0]) {
				
				sum_point += (i * 10000);
				
				sum_count++;
			}
		}
		
		if(sum_count < 1) {
			line_point_value[j][0] = INDEX_NOT_FOUND;
		}
		else 
			line_point_value[j][0] = sum_point / sum_count / 10000;
		
		if(j != 0) { // only in main camera
			if(sum_count > MAX_BLACK_COUNT) {
//				need_speed_down = true;
//				line_point_value[j][0] = INDEX_NOT_FOUND;
				schoolzone_count++;
			}
//			else {
//				need_speed_down = false;
//			}
		}
	}
	
	if(schoolzone_count >= 2) {
		line_point_value[0][0] = INDEX_NOT_FOUND;
		line_point_value[1][0] = INDEX_NOT_FOUND;
		line_point_value[2][0] = INDEX_NOT_FOUND;
		
		need_speed_down = true;
	}
	else {
		need_speed_down = false;
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
			
//			line_values[0][i] = 1023;// ignore it
			line_values[0][i] = A2D_GetSingleCh_10bit(PIN_LINE_CAM_1_ADC);
			line_values[1][i] = A2D_GetSingleCh_10bit(PIN_LINE_CAM_2_ADC);
			line_values[2][i] = A2D_GetSingleCh_10bit(PIN_LINE_CAM_3_ADC);
					
		}

		if(!is_started() /*&& draw_mode == DrawCamera*/) {
			
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
				
				line_max_min_table[2][i][CAM_MAX_VALUE_INDEX] = line_values[2][i];
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
