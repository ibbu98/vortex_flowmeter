/*
 * NHD_0216HZ.h
 *
 *  Created on: Sep 14, 2025
 *      Author: Damini
 */

#ifndef INC_NHD_0216HZ_H_
#define INC_NHD_0216HZ_H_

#include <stdint.h>

//Define constants
#define ENABLE 0x08   //enable bit for LCD latch
#define DATA_MODE 0x04   //RS=1 data register
#define COMMAND_MODE 0x00  //RS=0 instrument register

#define LINE_LENGTH 0x40  // Address offset for row 2
#define TOT_LENGTH 0x80   //Set DDRAM address
#ifdef ENABLE_SPI
//writing command to LCD
void write_cmd(uint8_t data);

//writing data to LCD
void write_data(char c);

//writing 4bit data/command via SPI
void write_4bit(uint8_t nibble, uint8_t mode);

//SPI initialization
void init_spi(void);

//LCD initialization
void init_lcd(void);

//set the cursor at a particular row and column
void set_cursor(uint8_t column, uint8_t row);

//print string on the LCD screen
void print_lcd(const char *string);
#endif


#endif /* INC_NHD_0216HZ_H_ */
