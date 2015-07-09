/*
 * servo_motor.c
 *
 *  Created on: Jun 30, 2015
 *      Author: luaviskang
 */

#include "servo_motor.h"
#include "typedefs.h"
#include "jdp.h"
#include "common.h"


static ServoMotor handle_motor;

/*
 * 
 * user parameter
 * 
 * max: 35
 * min: -35
 * 
 * real
 * center: 100
 * max : 135
 * min : 65
 * 
 */

#define FUTABA_S3010_MAX		2645
#define FUTABA_S3010_MIN		390
#define FUTABA_S3010_MIDDLE		1512

void servo_motor_init() {
	ServoMotor_init(&handle_motor, EMIOS_0_SERVO_MOTOR);
}
void servo_motor_move(int angle) {
	ServoMotor_rotate(&handle_motor, angle);
}

void ServoMotor_init(ServoMotor * servo, pinNum emios_channel) {
	
	servo->angle = 0;
	
	servo->emios_channel = emios_channel;
}

void ServoMotor_rotate(ServoMotor * servo, int angle) {
	
	servo->angle = angle;
	
	angle += 100;
	
	if(angle > 135)
		angle = 135;
	else if(angle < 65)
		angle = 65;
	
	EMIOS_0.CH[servo->emios_channel].CADR.R = (unsigned long int)(FUTABA_S3010_MIN + 11 * angle);
}
