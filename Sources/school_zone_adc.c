/*
 * school_zone_adc.c
 *
 *  Created on: Jul 13, 2015
 *      Author: luaviskang
 */


#include "school_zone_adc.h"
#include "common.h"
#include "adc_drv.h"
#include "rappid_utils.h"

#define ADC_BLACK_REFERENCE 500

bool is_school_zone_detected() {
	
	char buf[10];
	bool ret_val = false;
	
	DisableExternalInterrupts();
	
	int a1 = A2D_GetSingleCh_10bit(PIN_IR_SENSOR_1_ADC);
	
	int a3 = A2D_GetSingleCh_10bit(PIN_IR_SENSOR_3_ADC);
	
	int a5 = A2D_GetSingleCh_10bit(PIN_IR_SENSOR_5_ADC);
	
	if(a1 < ADC_BLACK_REFERENCE) {
		
		if(a5 < ADC_BLACK_REFERENCE) {
			
			if(a3 < ADC_BLACK_REFERENCE) {
				ret_val = true;
			}
		}
	}
	
	EnableExternalInterrupts();
	
	return ret_val;
}
