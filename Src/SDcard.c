/**
 * @file SDcard.c
 * @author Jakub Wrzosek
 * @date 23.05.2019
 * @brief Plik obslugujacy zewnetrzna karte SD.
 */

/* Includes ------------------------------------------------------------------*/

#include "SDcard.h"

#define FILE_NAME "pmik.txt"

//extern SD_HandleTypeDef hsd;

/* Private variables ---------------------------------------------------------*/
FRESULT res;			/**< Typ wyliczeniowy zwracajacy informacje o ewentualnych bledach */
FATFS SDFatFs;          /**< File system object structure (FATFS) */
FIL myFile;             /**< File object structure (FIL) */

uint8_t bsp_res;		/**< Zmienna do przechowywania rezultatu inicjalizacji karty SD */

char buffwr[100]; 		/**< Bufor danych do zapisu */
char buffrd[100]; 		/**< Bufor danych do odczytu */
char filePath[20];		/**< Sciezka do pliku, na ktorym chcemy wykonywac operacje */

uint32_t byteswr;		/**< Zmienna przechowujaca ilosc bajtow do zapisu */
uint32_t bytesrd;		/**< Zmienna przechowujaca ilosc odczytanych bajtow */

uint8_t err = 0;

/* Functions -----------------------------------------------------------------*/
void Error1(void){
	err++;
}

/**
 * @brief Funckja inicjalizujaca karte sd.
 */
void sd_start(void) 
{
	bsp_res = BSP_SD_Init();
	if(bsp_res != FR_OK){
		Error1();
	}
	
	res = f_mount(&SDFatFs, "", 1);
	if(res != FR_OK){
		Error1();
	}
	
	res = f_open(&myFile, FILE_NAME, FA_OPEN_ALWAYS|FA_WRITE|FA_READ);
	if(res != FR_OK){
		Error1();
	}
	
	res = f_lseek(&myFile, f_size(&myFile));
	if(res != FR_OK){
		Error1();
	}
	
	f_printf(&myFile, "Data:\t\t\tCzas:\t\t\tTemp:\t\t\tRh:\n\r");
}

/**
 * @brief Funkcja zapisujaca tablice znakow do kolejnego wolnego wiersza na karcie sd
 */
void sd_write(char *str)
{
	f_printf(&myFile, "%s", str);
}

/**
 * @brief Funkcja zwalniajaca karte sd przed zakonczeniem jej pracy (np. przed wyjeciem karty z gniazda)
 */
void sd_end(void)
{
	f_close(&myFile);
	f_mount(0, "", 1);
}



