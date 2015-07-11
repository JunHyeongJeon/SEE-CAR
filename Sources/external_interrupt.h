/*
 * external_interrupt.h
 *
 *  Created on: Jul 10, 2015
 *      Author: Jun
 */

#ifndef EXTERNAL_INTERRUPT_H_
#define EXTERNAL_INTERRUPT_H_

#include "common.h"
#include "car_mode.h"

#define NONE 0
#define BACK 1
#define INCREASE 2
#define DECREASE 3
#define SELECT 4


void siu_external_irq_0(void);

typedef uint8_t button;
static button push_button;
void button_increase();
void button_decrease();
void button_back();
void button_select();



#endif /* EXTERNAL_INTERRUPT_H_ */
