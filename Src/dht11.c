/**
 * @file dht11.c
 * @author Jakub Wrzosek
 * @date 23.05.2019
 * @brief Plik zawiera funkcje oraz zmienne niezbedne do komunikacji systemu z czujnikiem DHT11.
 */

/* Includes ------------------------------------------------------------------*/
#include "dht11.h"


/* Private variables ---------------------------------------------------------*/
GPIO_InitTypeDef GPIO_InitStruct;

uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;

uint16_t sum, RH, TEMP;

uint8_t check = 0;

char sd_dht_data[30] = "";

/* Functions -----------------------------------------------------------------*/

/**
 * @brief Funckja ustawiajaca pin odpowiedzialny za komunikacje z czujnikiem DHT11 jako wyjscie.
 */
void set_gpio_output(void)
{
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
 * @brief Funckja ustawiajaca pin odpowiedzialny za komunikacje z czujnikiem DHT11 jako wejscie.
 */
void set_gpio_input(void)
{
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
 * @brief Funckja przygotowujaca czujnik DHT11 do startu.
 */
void dht_start(void)
{
	/** Ustawiamy Pin jako wyjscie. */
	set_gpio_output(); // set pin as output
	/** Ustawiamy na pinie stan niski. */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); 
	/** Czekamy przez 18ms. */
	DWT_Delay_us(18000); 
	/** Ustawiamy Pin jako wejscie. */
	set_gpio_input();
}

/**
 * @brief Funckja sprawdzajaca gotowosc czujnika DHT11 do wymiany informacji.
 */
void check_response(void)
{
	/** Czekamy 40 us. */
	DWT_Delay_us(40);
 
	/** Jesli pin ma stan niski, czekamy 80us. */
	if(!(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)))
	{
		DWT_Delay_us(80);
		/** Ponownie sprawdzamy stan pinu. Jesli jest wysoki tzn. ze czujnik odpowiedzial. */
		if((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1))) check = 1; // now if the pin is high response = ok i.e. check = 1
	}
	/** Nastepnie czekamy az pin przyjmie stan niski. */
	while((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1))); // wait for the pin to go low
}

/**
 * @brief Funckja odbierajaca dane od czujnika DHT11.
 * @retval uint8_t Bajt danych od czujnika.
 */
uint8_t read_data(void)
{
	uint8_t i,j;
	for(j=0; j<8; j++)
	{
		while (!(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1))); // wait for the pin to go high

		DWT_Delay_us(40); // wait for 40us
		if((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)) == 0) // if the pin is low
		{
			i&= ~(1<<(7-j)); // write 0
		}
		else
		{
			i|= (1<<(7-j)); // if the pin is high, write 1
		}
		while((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1))); // wait for the pin to go low
	}
	return i;
}

/**
 * @brief Funckja integrujaca caly proces pomiaru. Inicjalizuje prace czujnika, sprawdza odpowiedz oraz dokonuje odczytu danych przesylanych przez czujnik DHT11.
 */
void dht11_makeMeas(void)
{
	dht_start ();
	check_response ();

	Rh_byte1 = read_data ();
	Rh_byte2 = read_data ();
	Temp_byte1 = read_data ();
	Temp_byte2 = read_data ();

	sum = read_data();

	if (sum == (Rh_byte1+Rh_byte2+Temp_byte1+Temp_byte2))    // if the data is correct
	{
		lcd_send_cmd (0x80);
		lcd_send_string ("TEMP:- ");
		lcd_send_data ((Temp_byte1/10)+48);
		lcd_send_data ((Temp_byte1%10)+48);
		lcd_send_string (" C");
			
		lcd_send_cmd (0xC0);
		lcd_send_string ("RH:- ");
		lcd_send_data ((Rh_byte1/10)+48);
		lcd_send_data ((Rh_byte1%10)+48);
		lcd_send_data ('%');
	}
}

/**
 * @brief Funckja przygotowujaca tablice znakow zawierajaca odpowiednio skonfigurowane dane do zapisu na karte SD
 * @retval char * tablica znakow zawierajaca odpowiednio skonfigurowane dane do zapisu na karte SD
 */
char * getDhtValues(void)
{
	sprintf(sd_dht_data, "%c%c\t\t\t%c%c\n\r", (Temp_byte1/10)+48, (Temp_byte1%10)+48, (Rh_byte1/10)+48, (Rh_byte1%10)+48);
	return sd_dht_data;
}





