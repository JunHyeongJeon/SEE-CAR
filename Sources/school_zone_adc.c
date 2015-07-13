/*
 * school_zone_adc.c
 *
 *  Created on: Jul 13, 2015
 *      Author: luaviskang
 */


#include "school_zone_adc.h"
#include "common.h"
#include "adc_drv.h"

#define ADC_BLACK_REFERENCE 500

bool is_school_zone_detected() {
	
	char buf[10];
	int a1 = A2D_GetSingleCh_10bit(PIN_IR_SENSOR_1_ADC);
	
	int a3 = A2D_GetSingleCh_10bit(PIN_IR_SENSOR_3_ADC);
	
	int a5 = A2D_GetSingleCh_10bit(PIN_IR_SENSOR_5_ADC);
	
	if(a1 < ADC_BLACK_REFERENCE) {
		
		if(a5 < ADC_BLACK_REFERENCE) {
			
			if(a3 < ADC_BLACK_REFERENCE) {
				return true;
			}
		}
		else {
			return false;
		}
	}
	else {
		return false;
	}
	
	return false;
}



//int IR_adc(int num){//num=0~4
//   
//   int adc_IR = 0;
//   
//   adc_IR = A2D_GetSingleCh_10bit(2*num+6);
//   
//   return adc_IR;
//}
//
//int SchoolZone_detected(){
//   int count=0;
//   int temp=0;
//   int i=0;
//   for(i=0;i<=4;i++){
//      temp=IR_adc(i);
//      if(temp<500)
//         count++;
//   }
//   if(count>=3)
//      return 1;
//   else
//      return 0;
//}
