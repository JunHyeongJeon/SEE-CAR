/*
 * common.c
 *
 *  Created on: Apr 8, 2015
 *      Author: see
 */

// for delay loop sec count

#include "common.h"
#include "UART_drv_api.h"
#include "line_scan.h"

unsigned long loops_per_sec = 16 * 1000 * 1000; // 64Mhz ?
__inline__ void __log(char * msg);

void writePin(pinNum pin, bool value) {
	// SIU.PCR[pin].R = PCR_OUTPUT;
	if(value < 1) value = 0;
	else value = 1;
	
	GPIO_SetState(pin, value);
}

bool readPin(pinNum pin) {
	//SIU.PCR[pin].R = PCR_INPUT;
	return GPIO_GetState(pin);
}

__inline__ void __delay(unsigned int loops)
{
	if (loops != 0)
		__asm__ __volatile__("mtctr %0; 1: bdnz 1b" : :
                                       "r" (loops) : "ctr");
}

void sdelay(unsigned long secs){
	int i;
	for (i=0; i<secs*10; i++){
		mdelay(100);
	}
}

void mdelay(unsigned long msecs)
{
	unsigned long loops;

	/* compute (usecs * 2^32 / 10^3) * loops_per_sec / 2^32 */
	msecs *= 0x418570;                /* 2^32 / 10^3 */
	
	__asm__("mulhwu %0,%1,%2" : "=r" (loops) :
			"r" (msecs), "r" (loops_per_sec));
 	__delay(loops);
}

void ndelay(unsigned long nsecs) {
	unsigned long loops;

	/* compute (usecs * 2^32 / 10^9) * loops_per_sec / 2^32 */
	nsecs *= 0x0004;                /* 2^32 / 10^9 */
	
	__asm__("mulhwu %0,%1,%2" : "=r" (loops) :
			"r" (nsecs), "r" (loops_per_sec));
	
 	__delay(loops);
}

void udelay(unsigned long usecs)
{
	unsigned long loops;

	/* compute (usecs * 2^32 / 10^9) * loops_per_sec / 2^32 */
	usecs *= 0x10c6;                /* 2^32 / 10^9 */
	
	__asm__("mulhwu %0,%1,%2" : "=r" (loops) :
			"r" (usecs), "r" (loops_per_sec));
 	__delay(loops);
 }

__inline__ void __log(char * msg) {
	println(msg);
}

void print(char * msg) {
	unsigned long str_ln = 0;
	char ch = msg[str_ln];
	
	while(ch) {
		str_ln++;
		ch = msg[str_ln];
	}
	
	UartTxMsg((unsigned char *) msg, str_ln);
}
void println(char * msg) {
	print(msg);
	UartTxMsg((unsigned char *) "\r\n", 2);
}

void sys_log(char * msg) {
	__log(msg);
}
void err_log(char * msg) {
	__log(msg);
}

long pow(long v, int n) {

  int i = 0;
  int ret = 1;

  if(n == 0) return 1;

  for(; i < n; i++) {
    ret *= v;
  }

  return ret;
}

int get_number_string_length(unsigned long val) {
	return get_number_length(val) + 1;
}

int get_number_length(unsigned long val) {
  int i = MAX_STOI_LENGTH;
  int cnt = 0;
  
  for(; i > 0; i--) {
    if((unsigned char) (val / pow(10, i)) != 0) {
      return i;
    }
    val = val % pow(10, i);
  }
  
  return 0;
}

void i_to_s_cnt(int val, char * ret, int cnt) {

  int num_length = cnt - 1;
  int i = num_length;
  bool is_val_signed = false;

  if(val < 0) {
    is_val_signed = true;
    val *= -1;
  }

  for(; i > 0; i--) {
    ret[num_length - i] = ((unsigned char) (val / pow(10, i - 1))) + '0';
    val = val % (unsigned int)pow(10, i - 1);
  }
  if(is_val_signed) {
    ret[0] = '-';
  }
  ret[num_length] = '\0';
}
void Uart_100(int flag) {
	unsigned char cDigit[3];
	cDigit[0] = (unsigned char) (flag / 100) + '0';
	flag = flag % 100;
	cDigit[1] = (unsigned char) (flag / 10) + '0';
	flag = flag % 10;
	cDigit[2] = (unsigned char) flag + '0';
	UartTxMsg(cDigit, 3);
	UartTxMsg((unsigned char *) "\r\n", 2);
}

void Uart_1000(int flag) {
	unsigned char cDigit[4];
	cDigit[0] = (unsigned char) (flag / 1000) + '0';
	flag = flag % 1000;
	cDigit[1] = (unsigned char) (flag / 100) + '0';
	flag = flag % 100;
	cDigit[2] = (unsigned char) (flag / 10) + '0';
	flag = flag % 10;
	cDigit[3] = (unsigned char) flag + '0';
	UartTxMsg(cDigit, 4);
	UartTxMsg((unsigned char *) "\r\n", 2);
}

void Uart_10000(int flag) {
	unsigned char cDigit[5];
	cDigit[0] = (unsigned char) (flag / 10000) + '0';
	flag = flag % 10000;
	cDigit[1] = (unsigned char) (flag / 1000) + '0';
	flag = flag % 1000;
	cDigit[2] = (unsigned char) (flag / 100) + '0';
	flag = flag % 100;
	cDigit[3] = (unsigned char) (flag / 10) + '0';
	flag = flag % 10;
	cDigit[4] = (unsigned char) flag + '0';
	UartTxMsg(cDigit, 5);
	UartTxMsg((unsigned char *) "\r\n", 2);
}

void clear_log_screen() {

	print("\033[2J"); // clear screen
	
	print("\033[0;0H"); // move to cursor top
}

#ifdef USE_CAM_1

static bool is_start = false;

bool is_started() {
	
	return is_start;
}

void start() {
	
	dbg_log("Start");
	
	is_start = true;
	make_avg_black();
}

#endif
