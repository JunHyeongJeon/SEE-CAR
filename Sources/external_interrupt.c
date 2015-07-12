/*
 * external_interrupt.c
#
 *
 *  Created on: Jul 10, 2015
 *      Author: Jun
 */
#include "external_interrupt.h"
#include "intc_pit.h"
#include "line_scan.h"

uint8_t m_page = 0;
char buf[10];

void siu_external_irq_0(void){

	if( pin_read (34) == 1){
		// Button First
		// Function Back
		button_back();
	//	sys_log("siu_external_34_BACK");
	}else if (pin_read(35) == 1){
		// Button Second
		// Function Left
		button_increase();
//		sys_log("siu_external_35_LEFT");
	}else if (pin_read(37) == 1){
		// Button Third
		// Function Right
		button_decrease();
	//	sys_log("siu_external_37_RIGHT");
	}else if (pin_read(14) == 1){
		// Button Fourth
		// Function Select
		button_select();
	//	sys_log("siu_external_14_SELECT");	
	}
	
	i_to_s_cnt(m_page, buf, 10);
	sys_log(buf);
	set_page(m_page);

			
	SIU.ISR.R = 0x000000ff;
}
void button_decrease(){
	if (m_page % 10 <= 0)
		return;
	m_page--;
}


void button_select(){
	uint8_t temp;
	temp = m_page % 10;
	m_page = temp * 10;
	
}
void button_back(){
	m_page = MAIN_PAGE;
}



