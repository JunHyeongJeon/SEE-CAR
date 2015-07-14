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

//#define FUTABA_S3010_MAX		670
//#define FUTABA_S3010_MIN		350

#define FUTABA_S3010_MIDDLE		490

void servo_motor_init() {
	ServoMotor_init(&handle_motor, EMIOS_0_SERVO_MOTOR);
}
void servo_motor_move(int angle, bool forced) {
	ServoMotor_rotate(&handle_motor, angle, forced);
}

int servo_get_current_angle() {
	
	return handle_motor.angle;
}

void ServoMotor_init(ServoMotor * servo, pinNum emios_channel) {
	
	servo->angle = 0;
	
	servo->emios_channel = emios_channel;
}

void ServoMotor_rotate(ServoMotor * servo, int angle, bool forced) {
	
#ifdef DEBUG
	char buf[10];
#endif
	
//	if(_abs(servo->angle - angle) > 50 && !forced) {
//		
//		return;
//	}
	
	servo->angle = angle;
	
	if(servo->angle > 40)
		servo->angle = 40;
	else if(servo->angle < -40)
		servo->angle = -40;
	
	if(servo->angle > 0) {
		EMIOS_0.CH[servo->emios_channel].CADR.R = (unsigned long int)(FUTABA_S3010_MIDDLE + (servo->angle * 166 / 45));
	}
	else {
		EMIOS_0.CH[servo->emios_channel].CADR.R = (unsigned long int)(FUTABA_S3010_MIDDLE + (servo->angle * 183 / 45));
	}
	
	
}
