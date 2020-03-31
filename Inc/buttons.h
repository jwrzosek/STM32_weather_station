/**
 * @file buttons.h
 * @author Jakub Wrzosek
 * @date 23.05.2019
 * @brief Plik naglowkowy pliku buttons.c
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include "i2c-lcd.h"

#include "stdbool.h"

#ifndef BUTTONS
#define BUTTONS


/* Functions prototypes -------------------------------------------------------*/

bool checkBouncing(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, bool *checkButton, bool *countingActivated ,uint8_t *counter);

void rightButton_HANDLER(void);


#endif /* BUTTONS */
