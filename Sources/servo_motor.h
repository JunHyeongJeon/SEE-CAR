/*
 * servo_motor.h
 *
 *  Created on: Jun 30, 2015
 *      Author: luaviskang
 */

#ifndef SERVO_MOTOR_H_
#define SERVO_MOTOR_H_

#include "common.h"

typedef struct {
	
	int angle;
	
	pinNum emios_channel;
} ServoMotor;

void ServoMotor_init(ServoMotor * servo, pinNum emios_channel);
void ServoMotor_rotate(ServoMotor * servo, int angle, bool forced);

extern ServoMotor handle_motor;

void servo_motor_init();
void servo_motor_move(int angle, bool forced);
int servo_get_current_angle();

#endif /* SERVO_MOTOR_H_ */
