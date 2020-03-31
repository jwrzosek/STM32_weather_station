/**
 * @file rtc.c
 * @author Jakub Wrzosek
 * @date 23.05.2019
 * @brief Plik obslugujacy wewnetrzny zegar czasu rzeczywistego (RTC) mikrokontrolera STM32F446RE.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "rtc.h"

extern RTC_HandleTypeDef hrtc;
/* Private variables ---------------------------------------------------------*/

uint8_t date[6];	/**< Tablica przechowujaca wartosci czasu oraz daty podczas procesu ich zmiany */

char changeTimeStatements[6][15] = {"Podaj godzine:", "Podaj minute: ", "Podaj sekunde:", "Podaj dzien:  ", "Podaj miesiac:", "Podaj rok:    "}; /**< Tablica przechowujaca komunikaty wyswietlane na wyswietlaczu podczas procesu zmiany czasu i daty */

uint8_t maxValue[6] = {23, 59, 59, 31, 12, 100};	/**< Tablica przechowujaca maksymalne mozliwe wartosci podczas procesu zmiany czasu i daty (godziny, minuty, sekundy, dni, miesiace, lata) */

uint8_t minValue[6] = {0, 0, 0, 1, 1, 0};	/**< Tablica przechowujaca minimalne mozliwe wartosci podczas procesu zmiany czasu i daty (godziny, minuty, sekundy, dni, miesiace, lata) */

uint8_t currentValue = 0; 	/**< Zmienna przechowujaca aktualnie ustawiana przez uzytkownika wartosc (godziny, minuty, sekundy, dni, miesiace, lata) */

char charValue[10] = "";	/**< Zmienna do ktorej konwertujemy wybierana przez uzytkownika wartosc w celu wyswietlenia jej na wyswietlaczu LCD */

extern bool checkChangeButton;

extern bool checkMinusButton;
extern bool checkPlusButton;
extern bool checkLeftButton;
extern bool tim13_startCounting;
extern uint8_t tim13_counter;

extern bool minus, plus, change, ok;

char rtc_time[12];	/**< Tablica przechowujaca lancuch znakow zawierajacy czas do wyswietlania na wyswietlaczu LCD */

char rtc_date[12];	/**< Tablica przechowujaca lancuch znakow zawierajacy date do wyswietlania na wyswietlaczu LCD */

char sd_rtc_data[40] = "";	/**< Tablica przechowujaca lancuch znakow zawierajacy sformatowany czas oraz date do zapisu na karte SD */

/* Functions -----------------------------------------------------------------*/

/**
 * @brief Plik obslugujacy zewnetrzna karte SD.
 * @param hour 		- godzina wybrana przez uzytkownika
 * @param minutes	- minuta wybrana przez uzytkownika
 * @param seconds 	- sekunda wybrana przez uzytkownika
 * @param day		- dzien wybrany przez uzytkownika
 * @param month		- miesiac wybrany przez uzytkownika
 * @param year		- rok wybrany przez uzytkownika
 */
void rtc_changeTime(uint8_t hour, uint8_t minutes, uint8_t seconds, uint8_t day,
					uint8_t month, uint8_t year)
{
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	
   /**Initialize RTC and set the Time and Date 
    */
  sTime.Hours = hour;
  sTime.Minutes = minutes;
  sTime.Seconds = seconds;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

  sDate.WeekDay = RTC_WEEKDAY_THURSDAY;
  sDate.Month = month;
  sDate.Date = day;
  sDate.Year = year;

	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}
  
  HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR1,0x32F2);
}

/**
 * @brief Funkcja odpowiadajaca za przeprowadzanie procedury zmiany czasu w RTC.
 */
void changeTime(void)
{
	uint8_t i = 0;
	lcd_send_cmd(LCD_CLEAR);
	
	while(checkChangeButton | change)
	{
		lcd_send_cmd(LCD_SET_FIRST_LINE);
		lcd_send_string(changeTimeStatements[i]);		// wysyla biezacy komunikat
		lcd_send_cmd(LCD_SET_SECOND_LINE);
		sprintf(charValue, "%d", currentValue);
		lcd_send_string(charValue);	// wysyla biezaca wartosc 
		
		if(checkMinusButton | minus){
			if( checkBouncing(MinusButton_GPIO_Port, MinusButton_Pin, &checkMinusButton, &tim13_startCounting, &tim13_counter) | minus ){
				minus = false;
				if(currentValue > minValue[i]) currentValue--;
			}
		}
		if(checkPlusButton | plus){
			if( checkBouncing(PlusButton_GPIO_Port, PlusButton_Pin, &checkPlusButton, &tim13_startCounting, &tim13_counter) | plus ){
				plus = false;
				if(currentValue < maxValue[i]) currentValue++;
			}
		}
		if(checkLeftButton | ok){
			if( checkBouncing(LeftButton_GPIO_Port, LeftButton_Pin, &checkLeftButton, &tim13_startCounting, &tim13_counter) | ok){
				ok = false;
				date[i] = currentValue;
				i++;
				currentValue = minValue[i];
				lcd_send_cmd(LCD_CLEAR);
			}
		} 	
		if(i == 6){
			checkChangeButton = false;
			change = false;
			rtc_changeTime(date[0], date[1], date[2], date[3], date[4], date[5]);
			sd_write("Zmieniono czas!!!\n\r");
		}
		
	}
	lcd_send_cmd(LCD_CLEAR);
}

/**
 * @brief Funckja ustawiajaca czas w RTC.
 */
void rtc_setTime(void)
{
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	
   /**Initialize RTC and set the Time and Date 
    */
  sTime.Hours = 0x12;
  sTime.Minutes = 0x00;
  sTime.Seconds = 0x00;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

  sDate.WeekDay = RTC_WEEKDAY_THURSDAY;
  sDate.Month = RTC_MONTH_APRIL;
  sDate.Date = 0x01;
  sDate.Year = 0x00;

	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}
  
  HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR1,0x32F2);
}

/**
 * @brief Funcka, za pomoca ktorej zapisujemy biezacy czas do zmiennych znakowych.
 */
void rtc_getTime(void)
{
	RTC_TimeTypeDef gTime;
	RTC_DateTypeDef gDate;
	
	/* Get the RTC current Time */
	HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
	
	/* Get the RTC current Date */
	HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);
	
	/* Display time Format: hh:mm:ss */
	sprintf((char*)rtc_time, "%02d:%02d:%02d", gTime.Hours, gTime.Minutes, gTime.Seconds);
	
	/* Display time Format: mm-dd-yy */
	sprintf((char*)rtc_date, "%02d.%02d.%02d", gDate.Date, gDate.Month, 2000 + gDate.Year);
}


/**
 * @brief Funckja, za pomoca ktorej mozmy ustawic alarm w RTC.
 */
void rtc_setAlarm(void)
{
	
	RTC_AlarmTypeDef sAlarm;
   /**Enable the Alarm A 
    */
  sAlarm.AlarmTime.Hours = 0x10;
  sAlarm.AlarmTime.Minutes = 0x21;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x12;
  sAlarm.Alarm = RTC_ALARM_A;
	
	if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

		HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR0,0x32F2);
}

/**
 * @brief Funckja wyswietlajaca biezacy czas i date na wyswietlaczu LCD.
 */
void rtc_displayTime(void)
{
	lcd_send_cmd(LCD_SET_FIRST_LINE);
	lcd_send_string(rtc_time);
	lcd_send_cmd(LCD_SET_SECOND_LINE);
	lcd_send_string(rtc_date);
}

/**
 * @brief Funckja przygotowujaca tablice znakow zawierajaca odpowiednio skonfigurowane dane do zapisu na karte SD
 * @retval char * tablica znakow zawierajaca odpowiednio skonfigurowane dane do zapisu na karte SD
 */
char * getRTCValues(void)
{
	sprintf(sd_rtc_data, "%s\t\t%s\t\t", rtc_date, rtc_time);
	return sd_rtc_data;
}




















