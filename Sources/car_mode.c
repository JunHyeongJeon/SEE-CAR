/*
 * car_mode.c
 *
 *  Created on: Jul 11, 2015
 *      Author: Jun
 */

#include "car_mode.h"

//char buf[10];

void set_page(uint8_t page){
	uint8_t scroll;
	
	scroll = page % 10;
	page = (page / 10) * 10;
	
	if( page == MAIN_PAGE){
		page_main(scroll);
	}else if( page == CHECK_SENSOR_VALUE_PAGE ){
		page_check_sensor_value(scroll);
	}else if( page == CHANGE_SENSOR_VALUE_PAGE){
		page_change_sensor_value(scroll);
	}else if ( page == START_CAR_PAGE){
		page_start_driving(scroll);
	}
}

void page_main(uint8_t scroll){
	sys_log("page_main");
	glcd_startScreen(scroll);
}
void page_check_sensor_value(uint8_t scroll){
	sys_log("page_check");
	glcd_checkSensorValueScreen(scroll);
}
void page_change_sensor_value(uint8_t scroll){
	sys_log("page_change");
	glcd_changeSensorValueScreen(scroll);
	
}
void page_start_driving(uint8_t scroll){
	sys_log("page_start");
	glcd_startCarScreen(scroll);
}



