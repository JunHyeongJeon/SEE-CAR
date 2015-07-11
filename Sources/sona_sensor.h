/*
 * sona_sensor.h
 *
 *  Created on: Jul 7, 2015
 *      Author: luaviskang
 */

#ifndef SONA_SENSOR_H_
#define SONA_SENSOR_H_

#include "common.h"

typedef struct {
	
	pinNum emios_channel;
	
	pinNum echo_pin_num;
	
	int recent_distance;
} SonaSensor;

static SonaSensor sona_sensor;

void sona_sensor_init();

void sona_sensor_send_echo();

void sona_sensor_end_echo();

int sona_sensor_get_pulse_width();

SonaSensor sona_sensor_get();

#endif /* SONA_SENSOR_H_ */
