/*
 * car_mode.h
 *
 *  Created on: Jul 11, 2015
 *      Author: Jun
 */

#ifndef CAR_MODE_H_
#define CAR_MODE_H_

#include "typedefs.h"
#include "st7565.h"
#define MAX_PAGE 4
#define MAX_SCROLL 4
#define NONE 0

enum page {
	MAIN_PAGE = 0,
	CHECK_SENSOR_VALUE_PAGE = 10,
	CHANGE_SENSOR_VALUE_PAGE = 20,
	START_CAR_PAGE = 30
};
enum main_page {
	CHECK_SENSOR_VALUE = 1,
	CHANGE_SENSOR_VALUE,
	START_CAR
};
enum check_sensor_value_page {
	CAMERA_FIRST = 11,
	CAMERA_SECOND,
	SONA_SENSOR,
	TILT_SENSOR
};
enum change_sensor_value_page {
	CAMERA_FIRST_CHANGE = 21,
	CAMERA_SECOND_CHANGE,
	SONA_SENSOR_CHANGE,
	TILT_SENSOR_CHANGE
};

void set_page(uint8_t);

void get_button_input();
uint8_t want_page_status();

void page_main(uint8_t);
void page_check_sensor_value(uint8_t);
void page_change_sensor_value(uint8_t);
void page_start_driving(uint8_t);

//uint8_t m_page_temp = 0;


#endif /* CAR_MODE_H_ */
