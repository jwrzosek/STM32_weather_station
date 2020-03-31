/**
 * @file main.c
 * @author Jakub Wrzosek
 * @date 23.05.2019
 * @brief Plik zawierajacy funkcje glowna main oraz konfiguracje peryferiow.
 */
/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "fatfs.h"

/* USER CODE BEGIN Includes */

#include "i2c-lcd.h"

#include "uart.h"

#include "dht11.h"

//#include "delay_us.h" ->  DO POPRAWY!

#include "dwt_stm32_delay.h"

#include "buttons.h"

#include "rtc.h"

#include "SDcard.h"

#include "stdbool.h"

#include "string.h"

#include "stdlib.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1; /**< Struktura zawierajaca konfiguracje i2c */

RTC_HandleTypeDef hrtc; /**< Struktura zawierajaca konfiguracje rtc */

SD_HandleTypeDef hsd; /**< Struktura zawierajaca konfiguracje sd */

TIM_HandleTypeDef htim10; /**< Struktura zawierajaca konfiguracje timera 10 */
TIM_HandleTypeDef htim13; /**< Struktura zawierajaca konfiguracje timera 13 */

UART_HandleTypeDef huart2; /**< Struktura zawierajaca konfiguracje uart */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

// ZMIENNE DO POMIARU DHT11:

bool getMeasure = false; /**< Jesli zmienna jest prawdziwa, wykonujemy pomiar czujnikiem DHT11 i wyswietlamy go na wyswietlaczu LCD. */

// ZMIENNE DO RTC:

bool displayTime = false; /**< Jesli zmienna jest prawdziwa, aktualizujemy godzine i date na lcd */


// ZMIENNE DO PRZYCISKOW:
/* Jesli zmienna jest prawdziwa tzn. ze otrzymalismy przerwanie od przycisku i rozpoczynamy procedure sprawdzenia poprawnosci zglaszanego 
 * przerwania poprzez cykliczne sprawdzanie stanu pinu w celu eliminacji bledu mozliwego ze wzgledu na drgania stykow. */
 
bool checkLeftButton = false; /**<  Jesli zmienna jest prawdziwa tzn. ze otrzymalismy przerwanie od przycisku LeftButton. */

bool checkRightButton = false; /**<  Jesli zmienna jest prawdziwa tzn. ze otrzymalismy przerwanie od przycisku RightButton. */

bool checkMinusButton = false; /**< Jesli zmienna jest prawdziwa tzn. ze otrzymalismy przerwanie od przycisku MinusButton . */

bool checkPlusButton = false; /**< Jesli zmienna jest prawdziwa tzn. ze otrzymalismy przerwanie od przycisku PlusButton. */

bool checkChangeButton = false; /**< Jesli zmienna jest prawdziwa tzn. ze otrzymalismy przerwanie od przycisku ChangeButton. */

bool wyswietlajCzas = false; /**< Jesli zmienna jest prawdziwa wyswietlaj czas, w przeciwnym razie wyswietlajtemperature. */

// ZMIENNE DO TIMERA 10 (1s):

uint8_t tim10_counter = 0; /**< Jesli zmienna jest prawdziwa rozpoczynamy zliczanie timerem 10 (1s) */

// ZMIENNE DO TIMERA 13 (10ms):

bool tim13_startCounting = false; /**< Jesli zmienna jest prawdziwa rozpoczynamy zliczanie timerem 13 (10ms) */

uint8_t tim13_counter = 0; /**< Ilosc zliczen timera 10ms */

bool isSdWorking = true; /**< Jesli zmienna = true to znaczy, ze karta SD jest inicjalizowana i zapisujemy na nia dane, w przeciwnym razie odmontowujemy karte (mozemy ja wyjac ze slotu. */
	
//Sterowanie przez uart
bool temp_czas = false; /**< Jesli zmienna jest prawdziwa ustawiamy flage oznaczajaca zmiane wyswietlanych na wyswietlaczu LCD danych z temeratury na czas. */
bool change = false; /**< Jesli zmienna jest prawdziwa to uruchamiamy tryb zmiany daty i czasu. */
bool minus = false; /**< Jesli stan zmiany czasu aktywny zmienna oznacza zmniejszenie wskazania wyswietlanego na wyswietlaczu o 1 */
bool plus = false; /**< Jesli stan zmiany czasu aktywny zmienna oznacza zwiekszenie wskazania wyswietlanego na wyswietlaczu o 1 */
bool ok = false; /**< Jesli stan zmiany czasu aktywny zmienna oznacza zaakceptowanie wskazania wyswietlanego na wyswietlaczu oraz przejscie do kolejnej sekcji */

uint8_t Received; /**< Znak odebrany za posrednictwem UARTu, na podstawie ktorego wykonujemy jedna z 6 dostepnych operacji */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM13_Init(void);
static void MX_I2C1_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM10_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
char * showINT(char* tab, uint8_t c)
{
	sprintf(tab, "Zegar: %i\n\r", c);
	return tab;
}

/** 
  * @brief Funkcja odpowiadajaca za obsluge przerwan pochodzacych od UARTu
  * @param huart - struktura przetrzymujaca informacje o UARTcie
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
 
 uint8_t data[50]; // Tablica przechowujaca wysylana wiadomosc.
 uint16_t size = 0; // Rozmiar wysylanej wiadomosci
 
/** Odebrany za pomoca UARTu znak przekazujemy do instrukcji warnkowej */
 switch (atoi((char *)&Received)) {
	
	/** Jesli odebrany znak to 1 - ustawiamy flage ktora sygnalizuje chec uzytkownika do zmiany rodzaju wyswietlanych danych. */
	case 1:
		//size = sprintf((char *)data, "[info]TimeTemp.\n\r");
		temp_czas = true;
		break;
	/** Jesli odebrany znak to 2 - zmieniamy wartos flagi mowiacej o wlaczeniu/wylaczeniu trybu zmiany daty/czasu. */
	case 2:
		if(change == false){
			change = true;
			//size = sprintf((char *)data, "[info]Change ON.\n\r");
		}else{ 
			change = false;
			checkChangeButton = false;
			size = sprintf((char *)data, "[info]Change OFF.\n\r");
		}
		break;
	/** Jesli odebrany znak to 3 - ustawiamy flage mowiaca o dekrementacji wartosci wybieranej przez uzytkownika w trybie zmiany daty/czasu.  */
	case 3:
		size = sprintf((char *)data, "[-]Minus.\n\r");
		minus = true;
		break;
	/** Jesli odebrany znak to 4 - ustawiamy flage mowiaca o inkrementacji wartosci wybieranej przez uzytkownika w trybie zmiany daty/czasu.  */
	case 4:
		size = sprintf((char *)data, "[+]Plus.\n\r");
		plus = true;
		break;
	/** Jesli odebrany znak to 5 - ustawiamy flage mowiaca o zaakceptowaniu wartosci wyswietlanej na wyswietlaczu LCD w trybie zmiany daty/czasu.  */
	case 5:
		size = sprintf((char *)data, "[info]Zatwierdzono.\n\r");
		ok = true;
		break;
	/** Jesli odebrany znak to 6 - ustawiamy flage mowiaca o zakonczeniu pracy karty SD i wykonujemy odmontowanie. */
	case 6:
		if (isSdWorking){
			size = sprintf((char *)data, "Mozna wyjac karte.\n\r");
			sd_end();
			HAL_GPIO_WritePin(RED_LED_EX_GPIO_Port, RED_LED_EX_Pin, GPIO_PIN_SET);
			isSdWorking = false;
		}else{
			size = sprintf((char *)data, "Mozna wlozyc karte.\n\r");
			sd_start();
			HAL_GPIO_WritePin(RED_LED_EX_GPIO_Port, RED_LED_EX_Pin, GPIO_PIN_RESET);
			isSdWorking = true;
		}
		break;
	/** Jesli odebrano niezdefiniowany znak wysylamy komunikat, iz otrzymany znak nie zostal rozpoznany przez urzadzenie. */
	default: 
		size = sprintf((char *)data, "Odebrano nieznany znak: %c\n\r", Received);
		break;
 }
 
 HAL_UART_Transmit_IT(&huart2, data, size); // Rozpoczecie nadawania danych z wykorzystaniem przerwan
 HAL_UART_Receive_IT(&huart2, &Received, 1); // Ponowne wlaczenie nasluchiwania
}

/** 
  * @brief Funkcja odpowiadajaca za obsluge przerwan zewnetrznych pochadzacych z portow GPIO 
  * @param GPIO_Pin - pin od ktorego otrzymalismy przerwanie
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/**	Jesli wcisnieto przycisk oznaczony symbolem R - ustawiamy flage symbolizujaca koniecznosc zmiany rodzaju wyswietlanych na wyswietlaczu LCD danych. */
	if(GPIO_Pin == RightButton_Pin)
	{
		checkRightButton = true;
	}
	
	/**	Jesli wcisnieto niebieski przycisk na plytce nucleo, ustawiamy flage symbolizujaca wejscie/wyjscie w stan zmiany daty/czasu. */
	if(GPIO_Pin == B1_Pin)
	{
		if(checkChangeButton == false)
		{
			checkChangeButton = true;
		} else { 
			checkChangeButton = false;
			change = false;
		}	
	}	
	
	/** Jesli wcisnieto przycisk oznaczony symbolem "-" - ustawiamy flage mowiaca o dekrementacji wartosci wybieranej przez uzytkownika w trybie zmiany daty/czasu.  */
	if(GPIO_Pin == MinusButton_Pin)
	{
		checkMinusButton = true;
	}
	
	/** Jesli wcisnieto przycisk oznaczony symbolem "+" - ustawiamy flage mowiaca o dekrementacji wartosci wybieranej przez uzytkownika w trybie zmiany daty/czasu.  */
	if(GPIO_Pin == PlusButton_Pin)
	{
		checkPlusButton = true;
	}
	
	/** Jesli wcisnieto przycisk oznaczony symbolem L - ustawiamy flage mowiaca o zaakceptowaniu wartosci wyswietlanej na wyswietlaczu LCD w trybie zmiany daty/czasu.  */
	if(GPIO_Pin == LeftButton_Pin)
	{
		checkLeftButton = true;
	}
}	
/* ---------------------------------------------------------------------------------- */

/** 
  * @brief Funkcja odpowiadajaca za obsluge przerwan od timerow.
  * @param htim - struktura przetrzymujaca informacje o timerach
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* Kod obslugi przerwania od timera 10, wywolywanego z czestotliwoscia 1Hz (1s)*/
	if(htim -> Instance == TIM10)
	{
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		getMeasure = true;		// ustawiamy flage zezwalajaca na pomiar
		displayTime = true;		// ustawiamy flage sygnalizujaca moment aktualizacji czasu i daty na wyswietlaczu
		
		if(isSdWorking == true)
		{
			if(tim10_counter >= 10){
				rtc_getTime();
				sd_write(getRTCValues());
				sd_write(getDhtValues());
				tim10_counter = 0;
			}
			tim10_counter++;
		}
	}
	/* Kod obslugi przerwania od timera 13, wywolywanego z czestotliwoscia 100Hz (10ms) */
	if(htim -> Instance == TIM13)
	{
		if(tim13_startCounting) {		// jesli zliczanie aktywowane
			tim13_counter++; 	// zwieksz ilosc zliczen
		}
	}
}
/* ---------------------------------------------------------------------------------- */

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

	/* USER CODE BEGIN 1 */
	
	/* USER CODE END 1 */
	
	/* MCU Configuration----------------------------------------------------------*/
	
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	
	/* USER CODE BEGIN Init */
	
	/* USER CODE END Init */
	
	/* Configure the system clock */
	SystemClock_Config();
	
	/* USER CODE BEGIN SysInit */
	
	/* USER CODE END SysInit */
	
	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_TIM13_Init();
	MX_I2C1_Init();
	MX_SDIO_SD_Init();
	MX_RTC_Init();
	MX_TIM10_Init();
	MX_FATFS_Init();
	
	/* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart2, &Received, 1); 
  
	HAL_TIM_Base_Start_IT(&htim10);
	HAL_TIM_Base_Start_IT(&htim13);
	DWT_Delay_Init ();
	
	rtc_setTime();
	
	lcd_init ();
	lcd_send_two_lines("Program","starting....");	// Linie wyswietlane przez 2s podczas startu programu
	HAL_Delay(2000);															// podczas startu programu.
	lcd_send_cmd(LCD_CLEAR);
	
	sd_start();
	/* USER CODE END 2 */
	
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
while (1)
  {
	// change type of displaying data
	if(checkRightButton | temp_czas){
		if( checkBouncing(RightButton_GPIO_Port, RightButton_Pin, &checkRightButton, &tim13_startCounting, &tim13_counter) | temp_czas ){
			if(temp_czas) uart_send_string("[info]Time/Temp.\n\r");
			rightButton_HANDLER();
			temp_czas = false;
		}
	}
	// change time mode	
	if(checkChangeButton | change){
		if(change) uart_send_string("[info]Change ON.\n\r");
		HAL_GPIO_WritePin(BLUE_LED_EX_GPIO_Port, BLUE_LED_EX_Pin, GPIO_PIN_SET);
		changeTime();
		HAL_GPIO_WritePin(BLUE_LED_EX_GPIO_Port, BLUE_LED_EX_Pin, GPIO_PIN_RESET);
		checkChangeButton = false;
		change = false;
	} 
	
	// jesli wyswietlaj czas = true to wyswietlaj czas, jesli nie wyswietlaj temperature
	if (wyswietlajCzas){
		if(displayTime){
			rtc_getTime();
			rtc_displayTime();
			displayTime = false;
		}
	}
	else{
		if(getMeasure){
			dht11_makeMeas();
			getMeasure = false;
		} 
	}
  /* USER CODE END WHILE */
  /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/** 
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_SDIO
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLQ;
  PeriphClkInitStruct.SdioClockSelection = RCC_SDIOCLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


/** 
  * @brief I2C1 init function 
  */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 10000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}


/** 
  * @brief RTC init function 
  */
static void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;
  RTC_AlarmTypeDef sAlarm;

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initialize RTC and set the Time and Date 
    */
  if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0x32F2){
  sTime.Hours = 0x10;
  sTime.Minutes = 0x20;
  sTime.Seconds = 0x30;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sDate.WeekDay = RTC_WEEKDAY_SUNDAY;
  sDate.Month = RTC_MONTH_JUNE;
  sDate.Date = 0x1;
  sDate.Year = 0x19;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR0,0x32F2);
  }
    /**Enable the Alarm A 
    */
  sAlarm.AlarmTime.Hours = 0x23;
  sAlarm.AlarmTime.Minutes = 0x59;
  sAlarm.AlarmTime.Seconds = 0x30;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}


/**
  * @brief SDIO init function 
  */
static void MX_SDIO_SD_Init(void)
{

  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 4;

}


/** 
  * @brief TIM10 init function 
  */
static void MX_TIM10_Init(void)
{

  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 9999;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 8399;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}


/** 
  * @brief TIM13 init function 
  */
static void MX_TIM13_Init(void)
{

  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 9999;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 41;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}


/**
  * @brief USART2 init function 
  */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}


/** 
  * @brief Configure pins as 
           * Analog 
           * Input 
           * Output
           * EVENT_OUT
           * EXTI
  */
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RED_LED_EX_GPIO_Port, RED_LED_EX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLUE_LED_EX_GPIO_Port, BLUE_LED_EX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RED_LED_EX_Pin */
  GPIO_InitStruct.Pin = RED_LED_EX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RED_LED_EX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BLUE_LED_EX_Pin */
  GPIO_InitStruct.Pin = BLUE_LED_EX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BLUE_LED_EX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MinusButton_Pin PlusButton_Pin ChangeButton_Pin RightButton_Pin 
                           LeftButton_Pin */
  GPIO_InitStruct.Pin = MinusButton_Pin|PlusButton_Pin|ChangeButton_Pin|RightButton_Pin 
                          |LeftButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
