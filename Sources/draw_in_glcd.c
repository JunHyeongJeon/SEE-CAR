/*
 * draw_in_glcd.c
 *
 *  Created on: Jul 15, 2015
 *      Author: Jun
 */
#include "draw_in_glcd.h"

extern bool is_school_zon_enable;
extern int sona_check_cut_line;
extern int pit_ch_2_ldval;
void sona_sensor_draw_in_glcd(){
	char buf[5];
	drawstring(0,0, "sona");
	i_to_s_cnt(sona_sensor_get().recent_distance, buf, 5);
	drawstring(0,1, buf);
	drawstring(0,2,"stop");
	i_to_s_cnt(sona_check_cut_line, buf, 5);
	drawstring(0,3, buf);
	glcd_display();
	
	glcd_small_clear();
}

void speed_draw_in_glcd(){
	char buf[5];
	drawstring(0,0, "speed");
	i_to_s_cnt(get_ref_speed(), buf, 5);
	drawstring(0,1, buf);
	glcd_display();
	
	glcd_small_clear();
	
}
void school_zone_draw_in_glcd(){
	char buf[5];
	drawstring(0,0, "szone");
	if( is_school_zon_enable)
		drawstring(0,1, "ON");
	else 
		drawstring(0,1,"OFF");
	
	glcd_display();
	
	glcd_small_clear();
		
}
void glcd_set_draw_in_glcd(){

	drawstring(0,0, "GLCD");
	drawstring(0,1, "switch");
	glcd_display();
	
	
	
	glcd_small_clear();
	
}

void glcd_si_draw_in_glcd(){
	char buf[10];
	drawstring(0,0, "sicut");
	
	i_to_s_cnt(pit_ch_2_ldval , buf, 10);

	drawstring(0, 1, buf);
	glcd_display();
	
	glcd_small_clear();
}


void line_scan_draw_in_glcd(int line_num, bool is_show_line_avg){
	
	
	int i;
	int j;
	int y_pos = 0;
	
	lineValue * line_values = line_values_get_index(line_num);
	
	int * detected_line_index = line_values_get_detected(line_num);
	
	for(i = 0; i < LINE_CAMERA_PIXEL_CONUT; i++ ){

		
		y_pos = line_values[i] / 16;
		
		if(is_show_line_avg) {
			setpixel(i, line_max_min_table[line_num][i][0] / 16, BLACK);
		}
		else {
			setpixel(i,y_pos,BLACK);	
		}
	}
	
	
	if(detected_line_index[0] != INDEX_NOT_FOUND)
		setpixel(detected_line_index[0], 0, BLACK);

	
	
	
	glcd_display();
	
	/* glcd screen clear */

	for(i = 0; i < LINE_CAMERA_PIXEL_CONUT; i++ ){
		
		y_pos = line_values[i] / 16;
		
		if(is_show_line_avg) {
			setpixel(i, line_max_min_table[line_num][i][0] / 16, WHITE);
		}
		else {
			setpixel(i,y_pos,WHITE);
		}
	}
	if(detected_line_index[0] != INDEX_NOT_FOUND)
			setpixel(detected_line_index[0], 0, WHITE);
			
	/* glcd screen clear */
	
}
