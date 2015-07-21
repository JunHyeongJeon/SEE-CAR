/*
 * intc_config.h
 *
 *  Created on: Jun 30, 2015
 *      Author: luaviskang
 */

#ifndef INTC_CONFIG_H_
#define INTC_CONFIG_H_

// PIT SEETING (handler in intc_pit)

#define PIT_BASE_CONTROL_CHANNEL 59

#define PIT_CAUTION_LIGHT_CHANNEL 0
#define PIT_SONA_SENSING_CHANNEL 1
#define PIT_AI_THINK_CHANNEL	 2
#define PIT_CAMERA_CHANNEL	 3

#define PIT_ENABLED(X) X != 0x00000000 ? 0x1 : 0x0

#define PIT_CHANNEL_0_TIMER 64000000 // light timer 1s
#define PIT_CHANNEL_1_TIMER	0x0009C400 // 10 ms; Initial value for start sona sensing
#define PIT_CHANNEL_2_TIMER	0x16000 //0x000001F4
//#define PIT_CHANNEL_2_TIMER	0x0001F400 // 1 ms
#define PIT_CHANNEL_3_TIMER	0x64000 * 2 // camera
#define PIT_CHANNEL_4_TIMER	0x00000000
#define PIT_CHANNEL_5_TIMER	0x00000000
#define PIT_CHANNEL_6_TIMER	0x00000000
#define PIT_CHANNEL_7_TIMER	0x00000000

// set timer time

#define PIT_SET_TIMER(X, Y) PIT.CH[X].LDVAL.R = Y; PIT.CH[X].TCTRL.B.TIE  = PIT_ENABLED(Y)

//start timer

#define PIT_START_TIMER_CHANNEL(X) PIT.CH[X].TCTRL.B.TEN = 0x01

// stop timer

#define PIT_STOP_TIMER_CHANNEL(X) PIT.CH[X].TCTRL.B.TEN = 0x00

// commit when timer get complete

#define PIT_COMMIT_TIMER(X) PIT.CH[X].TFLG.R = 1

#endif /* INTC_CONFIG_H_ */
