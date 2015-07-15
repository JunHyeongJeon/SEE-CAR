/*
 * line_scan.h
 *
 *  Created on: Jun 23, 2015
 *      Author: luaviskang
 */

#ifndef LINE_SCAN_H_
#define LINE_SCAN_H_

#include "common.h"

// Line camera 

typedef int lineValue;

typedef struct {
	
	pinNum serial_index_pin;
	pinNum clock_pin;
	pinNum adc_pin;
	
} LineCamera;



#define CAMERA_TOP 		0

#define CAMERA_LEFT	1

#define CAMERA_RIGHT	2 // NOT COFIGURED

#define DETECTED_LEFT 0
#define DETECTED_RIGHT 1

#define LINE_CAMERA_VALUE_LINE_COUNT LINE_CAMERA_COUNT

void LineCamera_init(LineCamera * camera, pinNum serial_index_pin, pinNum clock_pin, pinNum adc_pin);

// Application

static LineCamera lineCameras[LINE_CAMERA_COUNT];

static lineValue line_values[LINE_CAMERA_COUNT][LINE_CAMERA_PIXEL_CONUT];

static int line_point_value[LINE_CAMERA_VALUE_LINE_COUNT][2];

void line_scan_init();

void line_scan();

void line_calc();

int * line_values_get_detected(int index);
lineValue * line_values_get_index(int index);

#ifdef USE_CAM_1

#define CAM_MAX_VALUE_INDEX 0
#define CAM_MIN_VALUE_INDEX 1

bool is_need_to_speed_down(); // schoolzone or something

void make_avg_black(); // average black color
int get_avg_black();


#endif

void line_scan_draw_in_glcd(int line_num, bool is_show_line_avg);
void school_zone_draw_in_glcd();
void glcd_set_draw_in_glcd();
void glcd_si_draw_in_glcd();
//extern DRAW_MODE draw_mode;


#endif /* LINE_SCAN_H_ */
