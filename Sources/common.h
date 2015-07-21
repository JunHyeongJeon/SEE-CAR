/*
 * common.h
 *
 *  Created on: Apr 8, 2015
 *      Author: see
 */

#ifndef COMMON_H_
#define COMMON_H_

// general libraries

#include "gpio_drv.h"
#include "jdp.h"
#include "UART_drv_api.h"

#include "config.h"

#define INDEX_NOT_FOUND -1

#define DEBUG 1

#define USE_CAM_1
#ifdef USE_CAM_1
#define CAM_MAX_CUT_OFF 85 //%
#endif
// types
typedef unsigned char byte;
typedef unsigned short pinNum;
typedef unsigned char bool;
typedef char * string;

#define _abs(X) (X < 0 ? -1 * (X) : (X)) 

// general values

#define true 1
#define false 0

#define PCR_OUTPUT		0x0201
#define PCR_INPUT 		0x0101

#define _BV(bit) (1 << (bit))

#define MAX_STOI_LENGTH (sizeof(unsigned long) - 1)

// functions

void writePin(pinNum pin, bool value);
bool readPin(pinNum pin);

#define write_pin(X,Y) writePin(X, Y)

#define read_pin(X) readPin(X)
#define swap(a, b) { uint8_t t = a; a = b; b = t; }

void sdelay(unsigned long secs);
void mdelay(unsigned long msecs);
void udelay(unsigned long usecs); // micro seconds delay
void ndelay(unsigned long nsecs);

void print(char * msg);
void println(char * msg);

long pow(long v, int n);
int get_number_length(unsigned long val);
int get_number_string_length(unsigned long val);
void i_to_s_cnt(int val, char * ret, int cnt);

void sys_log(string msg);
void err_log(string msg);
void clear_log_screen();

#ifdef DEBUG
#define dbg_log(X) sys_log(X)
#else
#define dbg_log(X)
#endif

// deprecated

void Uart_100(int flag);
void Uart_1000(int flag);
void Uart_10000(int flag);


#ifdef USE_CAM_1
bool is_started();
void start();

typedef enum {
	DrawCamera = 0,
	DrawSona,
	DrawSpeed,
	DrawSchoolZone,
	DrawGlcdSet,
	DrawSiSet,
	DrawPSet,
	DrawISet,
	DrawDSet,
	DrawSchoolZoneSpeed,
	DrawBreakOn,
	DrawTurnSpeed,
} DRAW_MODE;

#endif

#endif /* COMMON_H_ */
