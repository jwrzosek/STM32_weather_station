/**
 * @file SDcard.c
 * @author Jakub Wrzosek
 * @date 23.05.2019
 * @brief Plik naglowkowy pliku SDcard.c
 */
 

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"
#include "fatfs.h"

#ifndef SDcard
#define SDcard


/* Functions prototypes -------------------------------------------------------*/
void Error1(void);


void sd_start(void);


void sd_write(char *str);


void sd_end(void);


#endif /* SDcard */
