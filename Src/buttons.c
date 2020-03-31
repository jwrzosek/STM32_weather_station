/**
 * @file buttons.c
 * @author Jakub Wrzosek
 * @date 23.05.2019
 * @brief Plik obslugujacy przyciski zewnetrzne (mini interfejs).
 */


/* Includes ------------------------------------------------------------------*/

#include "buttons.h"

/* Private variables ---------------------------------------------------------*/

extern TIM_HandleTypeDef htim13;

extern bool wyswietlajCzas;

/* Functions -----------------------------------------------------------------*/

 /** @brief  Funckja sprawdzajaca czy przycisk na pewno zostal wcisniety.
  * @param GPIOx - pod ktory port podlaczony przycisk ktory chcemy sprawdzic
  * @param GPIO_Pin - pod, ktory pin podlaczony przycisk ktory sprawdzamy
  * @param checkButton - flaga ustawiana przy wykryciu przerwania od przycisku
  * @param *countingActivater - flaga ustawiana w celu odliczania odpowiednich 
  *         okresow czasu, w ktorych upewniamy sie ze przycisk zostal wlaczony
  * @param counter - licznik 10ms odcinkow czasu w ktorych sprawdzany jest
  *        stan przycisku
  * @retval bool zwraca wartosc true - kiedy sprawdzenie przebiego pomyslnie, false
  *			- jesli uznalismy, ze przycisk nie zostal wcisniety
  */
bool checkBouncing(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, bool *checkButton, bool *countingActivated ,uint8_t *counter)
{
	/** Jesli stan niski na wejsciu niski rozpoczynamy cykliczne sprawdzanie w celu uniknieciu bledow spowodowanych drganiem stykow. */
	if(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_RESET) {
		*countingActivated = true;	
		if(*counter >= 4) {
			/** Jesli przycisk na pewno zostal wcisniety - zwracamy true. */
			*checkButton = false;
			*countingActivated = false;		
			*counter = 0;
			return true;
		}
	}
	/** Jesli procedura cyklicznego sprawdzania wykazala blad - zwracamy false */
	else {	
		*checkButton = false;
		*countingActivated = false;
		*counter = 0;
	}	 
	/** Jesli na wejsciu stan wysoki - zwracamy false. */
	return false;
}
		 
/** 
  * @brief  Funckja definiujaca zachowanie przycisku po wcisnieciu.
  */
void rightButton_HANDLER(void)
{
	/** Jesli flaga wyswietlajCzas - true, zmieniamy jej stan na false */
	if(wyswietlajCzas) {
		lcd_send_cmd(LCD_CLEAR);
		wyswietlajCzas = false;
	}
	/** Jesli flaga wyswietlajCzas - false, zmieniamy jej stan na true */
	else {
		lcd_send_cmd(LCD_CLEAR);
		wyswietlajCzas = true;
	}
}



