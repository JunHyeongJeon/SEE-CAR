/*
 * dc_motor.h
 *
 *  Created on: Jun 30, 2015
 *      Author: luaviskang
 */

#ifndef DC_MOTOR_H_
#define DC_MOTOR_H_

/*
 * 
 * H-BRIDGE_A-IN1 - left motor speed control (PTC3)
 * 
 * H-BRIDGE_A-IFB - direction (PTE23)
 * 
 * H-BRIDGE_ENABLE - enable(PTE21) J-10-3
 * 
 * H-BRIDGE_B-IN1 - right motor speed control (PTC2)
 * 
 * H-BRIDGE_B-IFB - direction (PTE22)
 * 
 */

#include "common.h"

typedef struct {
	
	unsigned long int duty_rate;
	
	pinNum emios_channel;
	
	pinNum reverse_pin;
	
	pinNum enabled_pin;
	
	bool is_reverse;
	
	bool is_run;
} Motor;


static Motor left_motor;
static Motor right_motor;

void Motor_init(Motor * m, pinNum emios_channel, pinNum reverse_pin, pinNum enabled_pin);
void Motor_set_speed(Motor * m, long int speed);

void dc_motor_init();

void dc_motor_left_set_duty_rate(long int duty_rate);

void dc_motor_right_set_duty_rate(long int duty_rate);

#endif /* DC_MOTOR_H_ */
