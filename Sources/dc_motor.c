/*
 * dc_motor.c
 *
 *  Created on: Jun 30, 2015
 *      Author: luaviskang
 */

#include "dc_motor.h"
#include "typedefs.h"
#include "jdp.h"
#include "gpio_drv.h"
#include "adc_drv.h"
#include "common.h"


void Motor_init(Motor * m, pinNum emios_channel, pinNum reverse_pin, pinNum enabled_pin) {
	m->duty_rate = 0;
	m->is_run = false;
	m->is_reverse= false;
			
	m->enabled_pin = enabled_pin;
	
	m->reverse_pin = reverse_pin;
	
	m->emios_channel = emios_channel;
}

void Motor_set_speed(Motor * m, long int speed) {

	write_pin(m->enabled_pin, true);
	
	if(speed >= 0) {
		m->is_reverse = false;
	}
	else {
		speed *= -1;
		m->is_reverse = true;
	}
	
	// limit
	
	if(speed > 1000)
		speed = 1000;
	
	m->duty_rate = speed;
	
	write_pin(m->reverse_pin, m->is_reverse);
	
	EMIOS_0.CH[m->emios_channel].CADR.R = (unsigned long int)m->duty_rate;

}

void dc_motor_init() {
	
	// right motor init
	
	Motor_init(&right_motor, EMIOS_0_RIGHT_MOTOR, PIN_RIGHT_MOTOR_REVERSE, PIN_MOTOR_ENABLE);
	
	// left motor init
	Motor_init(&left_motor, EMIOS_0_LEFT_MOTOR, PIN_LEFT_MOTOR_REVERSE, PIN_MOTOR_ENABLE);
}

void dc_motor_left_set_duty_rate(long int duty_rate) {
	Motor_set_speed(&left_motor, duty_rate);
}

void dc_motor_right_set_duty_rate(long int duty_rate) {
	Motor_set_speed(&right_motor, duty_rate);
}
