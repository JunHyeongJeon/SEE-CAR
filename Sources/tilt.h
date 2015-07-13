/*
 * tilt.h
 *
 *  Created on: Jul 13, 2015
 *      Author: luaviskang
 */

#ifndef TILT_H_
#define TILT_H_


typedef enum {
	NotLeaning,
	LeaningDown,
	LeaningUp
} LeaningMode;

LeaningMode get_tilt_senser();

#endif /* TILT_H_ */
