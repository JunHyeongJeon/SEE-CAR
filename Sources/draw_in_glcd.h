/*
 * draw_in_glcd.h
 *
 *  Created on: Jul 15, 2015
 *      Author: Jun
 */

#ifndef DRAW_IN_GLCD_H_
#define DRAW_IN_GLCD_H_

#include "common.h"
#include "st7565.h"
#include "sona_sensor.h"
#include "core_ai.h"
#include "line_scan.h"


void sona_sensor_draw_in_glcd();
void speed_draw_in_glcd();
void school_zone_draw_in_glcd();
void glcd_set_draw_in_glcd();
void glcd_si_draw_in_glcd();
void line_scan_draw_in_glcd(int line_num, bool is_show_line_avg);
void glcd_p_draw_in_glcd();
void glcd_i_draw_in_glcd();
void glcd_d_draw_in_glcd();
void draw_schoolzone_speed();

void draw_break_sp_in_glcd();

void draw_on_break_in_glcd();


#endif /* DRAW_IN_GLCD_H_ */
