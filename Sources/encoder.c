/*
 * encoder.c
 *
 *  Created on: Jun 30, 2015
 *      Author: luaviskang
 */

#include "encoder.h"
#include "adc_drv.h"

static Encoder left_encoder;

static Encoder right_encoder;

void Encoder_init(Encoder * encoder, int max_count, pinNum emios_channel, pinNum reverse_pin, int reference_value) {
	
	encoder->max_count = max_count;
	
	encoder->current_count = 0;
	
	encoder->prev_count = 0;
	
	encoder->prev_delta = 0;
	
	encoder->current_delta = 0;
		
	encoder->emios_channel = emios_channel;	
	
	encoder->reverse_pin = reverse_pin;
	
	encoder->reverse_reference_value = reference_value;
}

void Encoder_read(Encoder * encoder) {
	
	long en_num = 0;
	
	// read it
	
	en_num =  EMIOS_1.CH[encoder->emios_channel].CCNTR.R;
	
	if(en_num >= 65535) // it almost mean stop in true state
		en_num = 0;
	
	encoder->prev_count = encoder->current_count;
	encoder->current_count = en_num;
	
	encoder->prev_delta = encoder->current_delta;
	
	if(encoder->current_count > encoder->prev_count) { // checkcounter overflowed
		encoder->current_delta = encoder->current_count - encoder->prev_count;
	}
	else {
		encoder->current_delta = (encoder->max_count - encoder->prev_count) + encoder->current_count;
	}
	
	if(read_pin(encoder->reverse_pin) != encoder->reverse_reference_value)
		encoder->current_delta *= -1;
}

void encoder_init() {
	
	Encoder_init(&left_encoder, EMIOS_1_LEFT_ENCODER_MAX, EMIOS_1_LEFT_ENCODER, PIN_LEFT_ENCODER_REVERSE_PIN, LEFT_REVERSE_REF);
	
	Encoder_init(&right_encoder, EMIOS_1_RIGHT_ENCODER_MAX, EMIOS_1_RIGHT_ENCODER, PIN_RIGHT_ENCODER_REVERSE_PIN, RIGHT_REVERSE_REF);
}

long int encoder_read_left() {
	
	Encoder_read(&left_encoder);
		
	return left_encoder.current_delta;
}

long int encoder_read_right() {
	
	Encoder_read(&right_encoder);
	
	return right_encoder.current_delta;
}

Encoder get_left_encoder() {
	
	return left_encoder;
}

Encoder get_right_encoder() {
	
	return right_encoder;
}
