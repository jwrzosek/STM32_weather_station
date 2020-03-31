/**
 * @file uart.c
 * @author Jakub Wrzosek
 * @date 23.05.2019
 * @brief Plik naglowkowy pliku uart.c
 */


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include "string.h"

#ifndef UART_H
#define UART_H


/* Functions prototypes -------------------------------------------------------*/

void uart_send_string(char* s);

#endif /* UART */
