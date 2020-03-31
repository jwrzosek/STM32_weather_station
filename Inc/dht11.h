/**
 * @file dht11.h
 * @author Jakub Wrzosek
 * @date 23.05.2019
 * @brief Plik naglówkowy pliku dht11.c
 */
 

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"

#include "dwt_stm32_delay.h"

#include "i2c-lcd.h"

#include "stdbool.h"

#ifndef DHT11
#define DHT11

/* Functions prototypes -------------------------------------------------------*/


void set_gpio_output(void);

void set_gpio_input(void);

void dht_start(void);

void check_response(void);

uint8_t read_data(void);

void dht11_makeMeas(void);

char * getDhtValues(void);

#endif /* DHT11 */
