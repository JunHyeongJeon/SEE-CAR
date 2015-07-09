/*
 * encoder.h
 *
 *  Created on: Jun 30, 2015
 *      Author: luaviskang
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include "common.h"

#define ENCODER_ROTATION_RATE 800

#define ENCODER_ROTATION_CM_RATE 27 // ENCODER_DELTA_CM_RATE 27 cm

typedef struct {
	
	int max_count;
	
	int prev_count;
	
	int current_count;
	
	int prev_delta;
	
	int current_delta;
	
	pinNum emios_channel;
} Encoder;

void Encoder_init(Encoder * encoder, int max_count, pinNum emios_channel);

void Encoder_read(Encoder * encoder);

extern Encoder left_encoder;

extern Encoder right_encoder;

void encoder_init();

long int encoder_read_left();

long int encoder_read_right();

Encoder get_left_encoder();

Encoder get_right_encoder();

#endif /* ENCODER_H_ */
