/*
 * controls.h
 *
 *  Created on: Jun 30, 2015
 *      Author: luaviskang
 */

#ifndef CONTROLS_H_
#define CONTROLS_H_

typedef struct {
	
	// encoder speed
	
	long int wanted_speed;
	
	long int current_speed;
} Wheel;

void control_speed(Wheel * left, Wheel * right);

static Wheel left_wheel;
static Wheel right_wheel;


#endif /* CONTROLS_H_ */
