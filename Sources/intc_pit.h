/*
 *######################################################################
 *                                RAppIDJDP
 *           Rapid Application Initialization and Documentation Tool
 *                         Freescale Semiconductor Inc.
 *
 *######################################################################
 *
 * Project Name           : FSK_LED_Example
 *
 * Project File           : FSK_LED_Example.rsp
 *
 * Revision Number        : 1.0
 *
 * Tool Version           : 1.2.1.5
 *
 * file                   : intc_pit.h
 *
 * Target Compiler        : Codewarrior
 *
 * Target Part            : MPC5606B
 *
 * Part Errata Fixes      : none
 *
 * Project Last Save Date : 28-Oct-2013 16:04:30
 *
 * Created on Date        : 28-Oct-2013 16:04:31
 *
 * Brief Description      : This  file contains  the interrupt service routine  for the Periodic Interrupt Timer
 *
 *
 *######################################################################
*/

#ifndef  _INTC_PIT_H
#define  _INTC_PIT_H
/********************  Dependent Include files here **********************/

#include "jdp.h"

/**********************  Function Prototype here *************************/
static int line_draw_select;


void set_glcd_draw_select(int glcd_draw);

void toggle_glcd_draw_avg();
void toggle_glcd_draw_mode();

int get_draw_line_select();

void ai_control(void);

void sona_sensing(void);

void caution_light_handler(void);

void camera_proccess(void);

void sona_sensor_draw_in_glcd();
void speed_draw_in_glcd();
void school_zone_draw_in_glcd();
#endif  /*_INTC_PIT_H*/

/*
 *######################################################################
 *                           End of File
 *######################################################################
*/

