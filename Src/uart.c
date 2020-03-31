/**
 * @file uart.c
 * @author Jakub Wrzosek
 * @date 23.05.2019
 * @brief Plik obslugujacy komunikacje przez UART.
 */


/* Includes ------------------------------------------------------------------*/
#include "uart.h"

extern UART_HandleTypeDef huart2;
/* Private variables ---------------------------------------------------------*/


/* Functions -----------------------------------------------------------------*/

/**
  * @brief Funkcja przesyla tablice znakow przez UART.
  * @param char* wskaznik na tablice znakow, ktora chcemy przeslac.
  */
void uart_send_string(char* s)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), 1000);
}



