/*
 * external_interrupt.c
#
 *
 *  Created on: Jul 10, 2015
 *      Author: Jun
 */
#include "external_interrupt.h"

void siu_external_irq_0(void){
	mdelay(100);
	if( pin_read (34) == 1){
		sys_log("siu_external_0_34");
	}else if (pin_read(35) == 1){
		sys_log("siu_external_0_35");
	}else if (pin_read(37) == 1){
		sys_log("siu_external_0_37");
	}else if (pin_read(14) == 1){
		sys_log("siu_external_0_14");
		
	}

	SIU.ISR.R = 0x000000ff;

}


