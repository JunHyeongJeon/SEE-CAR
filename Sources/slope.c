/*
 * slope.c
 *
 *  Created on: Jul 13, 2015
 *      Author: luaviskang
 */

#include "slope.h"
#include "emios_init.h"
#include "common.h"

#define SLOPE_SONA_DOWN_COUNT 20
#define SLOPE_SONA_CLOSE_REFERENCES_WIDTH 500

static int slope_sona_sensor_count = 0;
static int recent_slope_sona_sensor = 0;

static SlopeState current_slope_state = SlopeGround;

void is_up_slope_detected();

int slope_sonar_get_pulse_width() {
	int current_distance = 0;
	
	// there is reason why multiple -1, Please Don't ask it :) 
	
	current_distance = EMIOS_0.CH[EMIOS_0_SLOPE_SONA_SENSOR].CBDR.B.CBDR - EMIOS_0.CH[EMIOS_0_SLOPE_SONA_SENSOR].CADR.B.CADR;
	current_distance *= -1;
	
	if(current_distance > 0) // error by overflow; ignore current count
		current_distance = recent_slope_sona_sensor;
	
	// if current distance is lower than sona close ref & it is going to close
	if(current_distance < SLOPE_SONA_CLOSE_REFERENCES_WIDTH &&
		current_distance <= recent_slope_sona_sensor) {  
		
		slope_sona_sensor_count++;
	}
	else {
		
		slope_sona_sensor_count = 0;
	}
	
	// initialize EMIOS A/B/COUNTER register
	
	EMIOS_0.CH[EMIOS_0_SLOPE_SONA_SENSOR].CBDR.B.CBDR = 0;
	
	EMIOS_0.CH[EMIOS_0_SLOPE_SONA_SENSOR].CADR.B.CADR = 0;
	
	EMIOS_0.CH[EMIOS_0_SLOPE_SONA_SENSOR].CCNTR.R = 0;
	
	EMIOS_0.CH[EMIOS_0_SLOPE_SONA_SENSOR].CCNTR.R = 0;
	EMIOS_0.CH[EMIOS_0_SLOPE_SONA_SENSOR].CSR.B.OVFL = 0;
	
	recent_slope_sona_sensor = current_distance;
	
	
	switch(current_slope_state) {
	case SlopeGround: // check it is
		
		is_up_slope_detected();
		
		break;
	default: 
		
		slope_sona_sensor_count = 0; // don't care it when not ground
		
		break;
	}
	
	return recent_slope_sona_sensor;
}

void is_up_slope_detected() {
	
	if(slope_sona_sensor_count > SLOPE_SONA_DOWN_COUNT) {
		
		current_slope_state = SlopeUpSide;
	}
}

SlopeState get_current_slope_state() {
	
	return current_slope_state;
}

void set_current_slope_state(SlopeState state) {
	
	current_slope_state = state;
}
