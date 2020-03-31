/**
 * @file rtc.h
 * @author Jakub Wrzosek
 * @date 23.05.2019
 * @brief Plik naglowkowy pliku rtc.c
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include "string.h"

#include "stdbool.h"

#include "buttons.h"

#include "i2c-lcd.h"

#include "SDcard.h"

#ifndef REALTIMECLOCK
#define REALTIMECLOCK


/* Functions prototypes -------------------------------------------------------*/

void rtc_changTime(uint8_t hour, uint8_t minutes, uint8_t seconds, uint8_t day,	uint8_t month, uint8_t year);


void changeTime(void);


void rtc_setTime(void);


void rtc_getTime(void);


void rtc_setAlarm(void);


void rtc_displayTime(void);


char * getRTCValues(void);

#endif /* REALTIMECLOCK */
