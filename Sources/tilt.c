/*
 * tilt.c
 *
 *  Created on: Jul 13, 2015
 *      Author: luaviskang
 */

#include "tilt.h"
#include "common.h"
#include "adc_drv.h"

#define LEANING_GROUND_DELTA 10

LeaningMode get_tilt_senser() {
	
	int adc_tilt = A2D_GetSingleCh_10bit(PIN_TILT_SENSER_ADC);
	char buf[10];
	
	i_to_s_cnt(adc_tilt, buf, 10);
	
	print("tilt");
	sys_log(buf);
	
//	if(adc_tilt < )
	return 0;
}
