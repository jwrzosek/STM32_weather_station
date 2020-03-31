/**
 * @file i2c-lcd.h
 * @author Jakub Wrzosek
 * @date 23.05.2019
 * @brief Plik naglowkowy pliku i2c-lcd.c
 */


#include "stm32f4xx_hal.h"

#define LCD_CLEAR 						0x01 
#define LCD_SET_FIRST_LINE 		0x80
#define LCD_SET_SECOND_LINE 	0xC0
#define LCD_DISPLAY_OFF				0x0A
#define LCD_DISPLAY_ON				0x0C

#ifndef I2C_LCD
#define I2C_LCD

/* Functions prototypes -------------------------------------------------------*/

void lcd_send_cmd (char cmd);  

void lcd_send_data (char data);  

void lcd_init (void);   

void lcd_send_string (char *str);  

void lcd_send_one_line(char *str);	

void lcd_send_two_lines(char *str1, char* str2);	


#endif /* I2C_LCD */

