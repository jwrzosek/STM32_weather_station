/**	@fie. delay_us.h
*	@date 05.03.2019
*   @author Jakub Wrzosek
*   @version 1.01
*	@brief Obsluga mikrosekundowego opoznienia niezbedna przy pracy z niektorymi peryferiami.
*/
 
 
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"


#ifndef DELAY_US
#define DELAY_US

/* Functions prototypes -------------------------------------------------------*/

void delay_us(uint16_t us);

#endif /* DELAY_US */
