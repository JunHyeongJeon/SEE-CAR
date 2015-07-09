///*
// * ks0108.c
// *
// *  Created on: Apr 14, 2015
// *      Author: luaviskang
// */
//
//// ks0108.cpp
//
//#include "ks0108.h"
//
//Coord coord_x = 0;
//Coord coord_y = 0;
//int lcd_page = 0;
//
//const uint8_t * lcd_font;
//uint8_t lcd_color;
//uint8_t font_color;
//
//#define DISPLAY_WIDTH	128
//#define DISPLAY_HEIGHT 	64
//#define CHIP_WIDTH		64
//int en_count = 0;
//
//#define EN_DELAY_VALUE 250
//#define EN_DELAY() ndelay(EN_DELAY_VALUE) // delay 1 usec
//
//#define PORT_DIR(PORT, MODE) SIU.PCR[PORT].R = MODE
//
//#define write_low(__pin__) writePin(__pin__, false)
//#define write_high(__pin__) writePin(__pin__, true)
//
//#define LCD_DATA_IN_LOW		read_data_to_byte() & 0b00001111
//#define LCD_DATA_IN_HIGH	read_data_to_byte() & 0b11110000
//
//static int ks0108_selected_chip = CS1;
//void ks0108_write_in_time(byte data, bool isCommand);
//
//void ks0108_set_address(byte y, byte chip);
//void ks0108_set_page(byte x, byte chip);
//
//byte chipSelect[] = {CS1, CS2};
//
//
//void ks0108_delay(int milisec) {
//	udelay((unsigned long)milisec * 1000);
//}
//
//void DATA_DIR_IN() {
//	PORT_DIR(D0, PCR_INPUT);
//	PORT_DIR(D1, PCR_INPUT);
//	PORT_DIR(D2, PCR_INPUT);
//	PORT_DIR(D3, PCR_INPUT);
//	PORT_DIR(D4, PCR_INPUT);
//	PORT_DIR(D5, PCR_INPUT);
//	PORT_DIR(D6, PCR_INPUT);
//	PORT_DIR(D7, PCR_INPUT);
//}
//
//void DATA_DIR_OUT() {
//	PORT_DIR(D0, PCR_OUTPUT);
//	PORT_DIR(D1, PCR_OUTPUT);
//	PORT_DIR(D2, PCR_OUTPUT);
//	PORT_DIR(D3, PCR_OUTPUT);
//	PORT_DIR(D4, PCR_OUTPUT);
//	PORT_DIR(D5, PCR_OUTPUT);
//	PORT_DIR(D6, PCR_OUTPUT);
//	PORT_DIR(D7, PCR_OUTPUT);
//}
//
//void ks0108_init_state() {
//	
//	uint8_t chip;
//	
//	PORT_DIR(DI, PCR_OUTPUT); 	
//    PORT_DIR(RW, PCR_OUTPUT); 	
//    PORT_DIR(E, PCR_OUTPUT); 	
//    PORT_DIR(CSA, PCR_OUTPUT);
//    PORT_DIR(CSB, PCR_OUTPUT);
//    PORT_DIR(RS, PCR_OUTPUT);
//    
//    ks0108_delay(10);
//    
//    writePin(E, false);
//    writePin(RW, false);
//    writePin(CS1, false);
//    writePin(CS2, false);
//    writePin(DI, false);
//    ks0108_control_display(true);
//    
//    ks0108_selected_chip = CSA;
//    ks0108_write_in_time(0xC0, true);
//    
//    ks0108_selected_chip = CSB;
//        ks0108_write_in_time(0xC0, true);
//}
//
//void ks0108_control_display(bool isOn) {
//	
//	byte command = 0b00111111;
//	if(!isOn)
//		command = 0b00111110;
//	
//	ks0108_selected_chip = CS1;
//	ks0108_write_in_time(command, true);
//	
//	ks0108_selected_chip = CS2;
//	ks0108_write_in_time(command, true);
//}
//
//void ks0108_write_in_time(byte data, bool isCommand) {
//	
//	DATA_DIR_OUT();
//	
//	writePin(E, false);
//	writePin(RW, false);
//	writePin(ks0108_selected_chip, true);
//	write_data_byte(0x00);
//	
//	if(isCommand)
//		writePin(DI, false);
//	else 
//		writePin(DI, true);
//	
//	writePin(E, true);
//	EN_DELAY();
//	EN_DELAY();
//	
//	writePin(E, false);
//	
//	write_data_byte(data);
//	EN_DELAY();
//	
//	writePin(E, true);
//	EN_DELAY();
//	EN_DELAY();
//	
//	writePin(E, false);
//	EN_DELAY();
//		
//	write_data_byte(0x00);
//	writePin(RW, false);
//	writePin(CS1, false);
//	writePin(CS2, true);
//	writePin(DI, false);
//}
//
//void ks0108_clear_screen(uint8_t color) {
//	int i, j = 0;
//	char a[2];
//	
////	for(j = 0; j < 8; j++)
////	{
//		ks0108_set_page(0, CSA);		
//		for(i = 0; i < 64; i++) {
//			ks0108_set_address(i, CSA);
//			ks0108_write_data(color);
//			udelay(500);
//		}
//		
//		i_to_s(j, a);
//		sys_log(a);
////	}
//}
//
//void ks0108_set_address(byte y, byte chip) {
//	y |= 0b01000000;
//	ks0108_write_command(y, chip);
//}
//
//void ks0108_set_page(byte page, byte chip) {
//	page |= 0b10111000;
//	ks0108_write_command(page, chip);
//}
//
//void ks0108_cursor_to(Coord x, Coord y) {
//	ks0108_go_to_coord( x * (* (lcd_font + FONT_FIXED_WIDTH) + 1),
//		       y * ( * (lcd_font + FONT_HEIGHT)+1)
//			 ) ; 
//}
//
//void ks0108_write_command(uint8_t cmd, uint8_t chip) {
//	ks0108_selected_chip = chip;
//	ks0108_write_in_time(cmd, true);
//}
//
//void ks0108_write_data(uint8_t data) {
//	ks0108_write_in_time(data, false);
//}
//
//void ks0108_select_font(const uint8_t* font, uint8_t color) {
//	lcd_font = font;
//	font_color = color;
//}
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//void ks0108_go_to_coord(Coord x, Coord y) {
//	uint8_t chip, cmd;
//	
//  if( (x > DISPLAY_WIDTH-1) || (y > DISPLAY_HEIGHT-1) )	// exit if coordinates are not legal
//    return;
//
//  coord_x = x;								// save new coordinates
//  coord_y = y;
//	
//  if(y / 8 != lcd_page) {
//  	lcd_page = y / 8;
//	cmd = LCD_SET_PAGE | lcd_page;			// set y address on all chips	
//	ks0108_write_command(cmd, CS1);
//	ks0108_write_command(cmd, CS2);
//  }
//
//  chip = coord_x / CHIP_WIDTH;
//  x = x % CHIP_WIDTH;
//  cmd = LCD_SET_ADD | x;
//  ks0108_write_command(cmd, chipSelect[chip]);					// set x address on active chip	
//}
//
//void ks0108_clear_page(uint8_t page, uint8_t color){
//	
//	uint8_t x=0;
//	
//	for(x = 0; x < DISPLAY_WIDTH; x++){	
//	   ks0108_go_to_coord(x, page * 8);
//	   ks0108_write_data(color);
//   }	
//}
//
//int ks0108_put_char(char c) {
//	uint8_t width = 0;
//	const uint8_t height = *(lcd_font + FONT_HEIGHT); // get font
//	uint8_t bytes = (height + 7) / 8;
//	
//	const uint8_t firstChar = *(lcd_font + FONT_FIRST_CHAR); // get font
//	const uint8_t charCount = *(lcd_font + FONT_CHAR_COUNT); // get font
//	
//	uint16_t index = 0;
//	uint8_t x = coord_x, y = coord_y;
//
//	uint8_t i;
//	uint8_t j;
//	uint8_t page;
//	uint8_t data;
//
//	if(c < firstChar || c >= (firstChar+charCount)) {
//		return 1;
//	}
//
//	c-= firstChar;
//
//	if( *(lcd_font+FONT_LENGTH) == 0 && *(lcd_font+FONT_LENGTH + 1) == 0) {
//    // zero length is flag indicating fixed width font (array does not contain width data entries)
//	   width = *(lcd_font+FONT_FIXED_WIDTH);
//	   index = c * bytes * width + FONT_WIDTH_TABLE;
//	}
//	else{
//	// variable width font, read width data, to get the index
//	   for(i = 0; i<c; i++) {  
//		 index += *(lcd_font + FONT_WIDTH_TABLE + i);
//	   }
//
//	   index = index * bytes +charCount+FONT_WIDTH_TABLE;
//	   width = *(lcd_font + FONT_WIDTH_TABLE + c);
//    }
//
//	// last but not least, draw the character
//	for(i = 0; i < bytes; i++) {
//		
//		page = i * width;
//
//		for(j = 0; j < width; j++) {
//			data = *(lcd_font + index + page + j);
//		
//			if(height > 8 && height < (i+1)*8) {
//				data >>= (i + 1) * 8 - height;
//			}
//			
//			if(font_color == KS0108_BLACK) {
//				ks0108_write_data(data);
//			} else {
//				ks0108_write_data(~data);
//			}
//		}
//		// 1px gap between chars
//		if(font_color == KS0108_BLACK) {
//			ks0108_write_data(0x00);
//		} else {
//			ks0108_write_data(0xFF);
//		}
//		ks0108_go_to_coord(x, coord_y + 8);
//	}
//
//	ks0108_go_to_coord(x + width + 1, y);
//
//	return 0;
//}
//
//uint8_t ks0108_do_read_data(uint8_t first) {
//	uint8_t data, chip;
//
//	chip = coord_x / CHIP_WIDTH;
//	ks0108_wait_ready(chip);
//
//	if(first){
//		if(coord_x % CHIP_WIDTH == 0 && chip > 0){ 		// todo , remove this test and call GotoXY always?
//		  ks0108_go_to_coord(coord_x, coord_y);
//		  ks0108_wait_ready(chip);
//		}
//	}
//
//	write_high(DI);					// D/I = 1
//	write_high(RW);					// R/W = 1
//	dbg_log("DI High");
//		
//	write_high(E); 					// EN high level width: min. 450ns
// 	EN_DELAY();
//
//#ifdef LCD_DATA_NIBBLES
//	 data = read_data_to_byte();
//	 
//#else
//	 data = LCD_DATA_IN_LOW;	   		// low and high nibbles on same port so read all 8 bits at once
//#endif 
//
//	write_low(E);
//
//    if(first == 0) 
//	  ks0108_go_to_coord(coord_x, coord_y);	
//	
//	return data;
//}
//
//__inline__ uint8_t ks0108_read_data() {
//	ks0108_do_read_data(1);				// dummy read
//	return ks0108_do_read_data(0);			// "real" read
//	coord_x++;
//}
//
//void ks0108_wait_ready(uint8_t chip) {
//	ks0108_select_chip(chip);
//	write_data_byte(0x00);
//
//	write_low(DI);
//	dbg_log("LOW~! - W");
//	
//	write_high(RW);
//	write_high(E);
//
//	EN_DELAY();
//
//	while(LCD_DATA_IN_HIGH & LCD_BUSY_FLAG){}
//	
//	write_low(E);
//}
//
//__inline__ void ks0108_select_chip(uint8_t chip) {  
////static uint8_t prevchip; 
//	if(chipSelect[chip] & 1)
//       write_high(CSA);
//	else
//	   write_low(CSA);
//
//	if(chipSelect[chip] & 2)
//       write_high(CSB);
//	else
//	   write_low(CSB);
//}
//
// __inline__ void ks0108_enable(void) {  
//   EN_DELAY();
//   write_high(E);					// EN high level width min 450 ns
//   EN_DELAY();
//   write_low(E);
//   //EN_DELAY(); // some displays may need this delay at the end of the enable pulse
//}
//
//__inline__ void write_data_byte(byte byt) {
//	writePin(D0, byt & 0B00000001);
//	writePin(D1, byt & 0B00000010);
//	writePin(D2, byt & 0B00000100);
//	writePin(D3, byt & 0B00001000);
//	writePin(D4, byt & 0B00010000);	
//	writePin(D5, byt & 0B00100000);
//	writePin(D6, byt & 0B01000000);
//	writePin(D7, byt & 0B10000000);
//}
//
//__inline__ byte read_data_to_byte() {
//	return readPin(D0) + 
//		   readPin(D1) * 2 +
//		   readPin(D2) * 4 +
//		   readPin(D3) * 8 +
//		   readPin(D4) * 16 +
//		   readPin(D5) * 32 +
//		   readPin(D6) * 64 +
//		   readPin(D7) * 128;
//}
