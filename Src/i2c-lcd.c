/**
 * @file i2c-lcd.c
 * @author Jakub Wrzosek
 * @date 23.05.2019
 * @brief Plik zawierajacy funkcje odpowiedzialne za komunikacje z wyswietlaczem LCD 2x16.
 */

/* Includes ------------------------------------------------------------------*/

#include "i2c-lcd.h"

#define SLAVE_ADDRESS_LCD 0x4E 

/* Private variables ---------------------------------------------------------*/

extern I2C_HandleTypeDef hi2c1;  

/* Functions -----------------------------------------------------------------*/

/**
 * @brief Funkcja przesylajaca komende do wyswietlacza przy nastepujaco ustawionych liniach en=1, rs=0, r/~w=0
 * @param cmd - kod komendy.
 */
void lcd_send_cmd (char cmd)
{
  char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xF0);
	data_l = ((cmd<<4)&0xF0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

/**
 * @brief Funkcja przesylajaca znak do wyswietlacza przy nastepujaco ustawionych liniach en=1, rs=1, r/~w=0
 * @param data - znak przesylany do wyswietlacza.
 */
void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xF0);				// upewniamy sie ze data na pewno bedzie takie jakie chcemy
	data_l = ((data<<4)&0xF0);	// upewniamy sie ze data na pewno bedzie takie jakie chcemy
	data_t[0] = data_u|0x0D;  	//en=1, rs=1
	data_t[1] = data_u|0x09; 	 	//en=0, rs=1
	data_t[2] = data_l|0x0D; 		//en=1, rs=1
	data_t[3] = data_l|0x09; 		//en=0, rs=1
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

/**
 * @brief Funcka inicjalizujaca wyswietlacz LCD. Przesyla w kojenosci niezbedne do inicjalizacji komendy.
 */
void lcd_init (void)
{
	lcd_send_cmd (0x02);	// return home
	lcd_send_cmd (0x28);	//
	lcd_send_cmd (0x0C);	// display on, cursor off
	lcd_send_cmd (0x80);	// force coursor to beginning (1st line)
}

/**
 * @brief Funkcja przesylajaca do wyswietlacza ciag znakow.
 * @param str - wskaznik na tablice znakow.
 */
void lcd_send_string (char *str)
{
	while (*str) 
		lcd_send_data (*str++);
}

/**
 * @brief Funkcja przesylajaca do wyswietlacza jedna linie znakow (uprzenio czyszczac wyswietlacz).
 * @param str - wskaznik na tablice znakow.
 */
void lcd_send_one_line(char *str)
{
	lcd_send_cmd(LCD_CLEAR);
	lcd_send_string(str);
}

/**
 * @brief Funkcja przesylajaca do wyswietlacza dwie linie znakow (uprzenio czyszczac wyswietlacz).
 * @param str1 - wskaznik na ciag znakow do wyswietlenia w pierwszej linii wyswietlacza.
 * @param str1 - wskaznik na ciag znakow do wyswietlenia w drugiej linii wyswietlacza.
 */
void lcd_send_two_lines(char *str1, char* str2)
{
	lcd_send_cmd(LCD_CLEAR);
	
	lcd_send_cmd(LCD_SET_FIRST_LINE);
	lcd_send_string(str1);
	
	lcd_send_cmd(LCD_SET_SECOND_LINE);
	lcd_send_string(str2);
}



























