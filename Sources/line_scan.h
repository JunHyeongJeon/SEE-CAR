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

#define CAMERA_MIDDLE	1

#define CAMERA_BOTTOM	-1 // NOT COFIGURED

#define DETECTED_LEFT 0
#define DETECTED_RIGHT 1

#define LINE_CAMERA_VALUE_LINE_COUNT LINE_CAMERA_COUNT - 1

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

static lineValue line_max_min_table[LINE_CAMERA_PIXEL_CONUT][2];
lineValue * line_values_get_max_min(int index);
#endif

void line_scan_draw_in_glcd(int line_num);

#endif /* LINE_SCAN_H_ */
