/*
 * slope.h
 *
 *  Created on: Jul 13, 2015
 *      Author: luaviskang
 */

#ifndef SLOPE_H_
#define SLOPE_H_

typedef enum {
	SlopeGround,
	SlopeUpSide,
	SlopeDownSide,
} SlopeState;

int slope_sonar_get_pulse_width();

SlopeState get_current_slope_state();

void set_current_slope_state(SlopeState state);

#endif /* SLOPE_H_ */
