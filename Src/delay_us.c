/** @file delay_us.c
*	@date 05.03.2019
*   @author Jakub Wrzosek
*   @version 1.01
*	@brief Obsluga mikrosekundowego opoznienia niezbedna przy pracy z niektorymi peryferiami.
*/
 
/* Includes ------------------------------------------------------------------*/
#include "delay_us.h"


// extern TIM_HandleTypeDef np.htim10;

/* Functions -----------------------------------------------------------------*/

/*
* Funkcja dziala przy odpowiednio skonfigurowanym timerze.
* Przed uzyciem nalezy skonfigurowac odpowiedni licznik,
* tak aby zliczal z odpowiednia czestotliwoscia.
*/
void delay_us(uint16_t us)
{
	//np.htim10.Instance->CNT = 0;
	
	//while(np.htim10.Instance->CNT <= us);
}
