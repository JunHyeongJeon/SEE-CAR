/*
 * sona_sensor.c
 *
 *  Created on: Jul 7, 2015
 *      Author: luaviskang
 */

#include "sona_sensor.h"

void sona_sensor_init() {
	
	sona_sensor.echo_pin_num = PIN_SONA_SENSOR_ECHO;
	
	sona_sensor.emios_channel = EMIOS_0_SONA_SENSOR;
	
	sona_sensor.recent_distance = 0;
	
}

void sona_sensor_send_echo() {
	
	//TODO : WORK WITH TIMER
	
	write_pin(sona_sensor.echo_pin_num, 0);
	write_pin(sona_sensor.echo_pin_num, 1);
	
}

void sona_sensor_end_echo() {
	
	write_pin(sona_sensor.echo_pin_num, 0);
}

int sona_sensor_get_pulse_width() {
	
	int recent_distance = 0;
	
	// there is reason why multiple -1, Please Don't ask it :) 
	
	recent_distance = EMIOS_0.CH[sona_sensor.emios_channel].CBDR.B.CBDR - EMIOS_0.CH[sona_sensor.emios_channel].CADR.B.CADR;
	recent_distance *= -1;
	
	if(recent_distance > 0) // error by overflow; ignore current count
		sona_sensor.recent_distance = recent_distance;
	
	// initialize EMIOS A/B/COUNTER register
	
	EMIOS_0.CH[sona_sensor.emios_channel].CBDR.B.CBDR = 0;
	
	EMIOS_0.CH[sona_sensor.emios_channel].CADR.B.CADR = 0;
	
	EMIOS_0.CH[sona_sensor.emios_channel].CCNTR.R = 0;
	
	EMIOS_0.CH[sona_sensor.emios_channel].CCNTR.R = 0;
	EMIOS_0.CH[sona_sensor.emios_channel].CSR.B.OVFL = 0;
	
	return sona_sensor.recent_distance;
}

SonaSensor sona_sensor_get() {
	
	return sona_sensor;
}
