#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lora.h"
#include "driver/gpio.h"

#define TAG_LORA "LoRa"
#define TAG_PROGRAM "Program"
#define TAG_GPS "GPS"

/*---- GPS ----*/
/* 
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "freertos/queue.h"
#include "soc/uart_struct.h"

#define TAG_PROGRAM "Program"
#define TAG_DISPLAY "Display"


#define ECHO_TEST_TXD (27)
#define ECHO_TEST_RXD (16)

*/


/*-- functions --*/
void init_lora();
void setup_lora();
void task_tx(void *p);
void gpio_init();
void gpio_read();

/*-- GPIO --*/
static const int BUTTON_PIN = 4;
static bool input_state = 0;

void app_main()
{
	gpio_init();
	init_lora();
}


/* ------------------------------ LORA ------------------------------------ */
void init_lora()
{
	setup_lora();
	xTaskCreate(&task_tx, "task_tx", 2048, NULL, 5, NULL);
}

void setup_lora()
{
	lora_init();
	lora_set_frequency(8681e5);
	lora_enable_crc();
	lora_set_spreading_factor(7);
	lora_set_bandwidth(125E3);
	lora_set_coding_rate(5);
	lora_set_preamble_length(8);
	lora_set_sync_word(0x12);
}

void task_tx(void *p)
{
	printf("Start Sending...");
	for (;;)
	{
		gpio_read();
		vTaskDelay(pdMS_TO_TICKS(100));
		if (input_state)
		{
			lora_send_packet((uint8_t *)"open", 8);
			printf("packet sent...\n");
			vTaskDelay(pdMS_TO_TICKS(5000));
		}
	}
}

/* ------------------------------ GPIO ------------------------------------ */
void gpio_read(){
	input_state = gpio_get_level(BUTTON_PIN);
}

void gpio_init()
{
	gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
}

/* ------------------------------ GPS ------------------------------------ */

/* 
char lGV[32];
char lRV[32];
char bGV[32];
char bRV[32];

void checkVariable(char *laengenGrad,char *laengenRichtung,char *breitenGrad,char *breitenRichtung) {
    if (strcmp(lGV, laengenGrad) == 0) {
        printf("laengenGrad:%s--%s", lGV, laengenGrad);
        if (strcmp(lRV, laengenRichtung) == 0) {
            printf("\nlaengenRichtung:%s--%s", lRV, laengenRichtung);
            if (strcmp(bGV, breitenGrad) == 0) {
                printf("\nbreitenGrad:%s--%s", bGV, breitenGrad);
                if (strcmp(bRV, breitenRichtung) == 0) {
                    printf("\nbreitenRichtung:%s--%s", bRV, breitenRichtung);

                    printf("\nGaragentor oeffnen lora senden\n");

					// xTaskCreate(&task_tx, "task_tx", 2048, NULL, 5, NULL);
                }
            }
        }
    }
}

void setVariable(char *laengenGrad,char *laengenRichtung,char *breitenGrad,char *breitenRichtung) {
    //Globalen Variablen durch eprom ersetzten
    strcpy(lGV, laengenGrad);
    strcpy(lRV, laengenRichtung);
    strcpy(bGV, breitenGrad);
    strcpy(bRV, breitenRichtung);
}

char *roundChar(char *string) {
    char *wertChar;
    double wertDouble = atof(string);

    wertDouble *= 1000;
    wertDouble = round(wertDouble);
    wertDouble /= 1000;
	char arr[sizeof(wertDouble)];
	memcpy(arr,&wertDouble,sizeof(wertDouble));
	wertChar = arr;
    return wertChar;
}

void readGPS(char *base)
{
	char newLaengenGrad[32];
    char newBreitenGrad[32];
	char *lG;
    char *bG;

	printf("Abschnitt gefunden: %s\n", base);
	char str[64], laengenGrad[32], laengenRichtung[32], breitenGrad[32], breitenRichtung[32], zeit[32], check1[32], check2[32];
	base += 5;
	strcpy(str, base);

	strcpy(laengenGrad, strtok(str, ","));
	strcpy(laengenRichtung, strtok(NULL, ","));
	strcpy(breitenGrad, strtok(NULL, ","));
	strcpy(breitenRichtung, strtok(NULL, ","));
	strcpy(zeit, strtok(NULL, ","));
	strcpy(check1, strtok(NULL, ","));
	strcpy(check2, strtok(NULL, ","));

	printf("Laengengrad: %s\n", laengenGrad);
	printf("Laengen Richtung: %s\n", laengenRichtung);
	printf("Breitengrad: %s\n", breitenGrad);
	printf("Breiten Richtung: %s\n", breitenRichtung);
	printf("Zeit: %s\n", zeit);
	printf("Check1: %s\n", check1);
	printf("Check2: %s\n", check2);



    strcpy(newLaengenGrad, roundChar(laengenGrad));
    strcpy(newBreitenGrad, roundChar(breitenGrad));

    lG = newLaengenGrad;
    lG[strlen(lG)-3] = 0;
    bG = newBreitenGrad;
    bG[strlen(bG)-3] = 0;
	// checkVariable(newLaengenGrad, laengenRichtung, newBreitenGrad, breitenRichtung);
}

static void echo_task()
{
	
	const int uart_num = UART_NUM_1;
	uart_config_t uart_config = {
		.baud_rate = 9600,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.rx_flow_ctrl_thresh = 122,
	};
	// Configure UART1 parameters
	uart_param_config(uart_num, &uart_config);

	uart_set_pin(uart_num, ECHO_TEST_TXD, ECHO_TEST_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, 0);

	uint8_t data[BUF_SIZE] = {0};

	char *data_char = (char *)data;
	

	while (1)
	{
		test_GPS(uart_num, data_char);

		vTaskDelay(1000 / portTICK_RATE_MS);
	}
}

void test_GPS(int uart_num, char *data){
		// Read data from UART
		int len = uart_read_bytes(uart_num, data, BUF_SIZE, 20 / portTICK_RATE_MS);
		printf("Length: %d\n", len);
		if (len > 0)
		{
			char *string;
			string = (char *)data;
			char delimiter[] = "\n";
			char *ptr, *gpgl, *gpgll;

			// initialisieren und ersten Abschnitt erstellen
			ptr = strtok(string, delimiter);
			printf(ptr);


			while (ptr != NULL)
			{
				gpgll = strstr(ptr, "$GPGLL");
				if (!gpgll)
				{
					gpgl = strstr(ptr, "$GPGL");
					if (gpgl)
					{
						// readGPS(ptr);
						printf("Found GPGL\n");
					}	
				} else {
					// int gpLength = strlen(ptr);
					gpgll = strstr(ptr, "A");
					// if (gpLength > 10){
						// printf("Found something: %s\n", gpgll);
					// }
					if (gpgll) {
						printf("Found Coordinates: %s\n", gpgll);
					}
				}
				// naechsten Abschnitt erstellen
				ptr = strtok(NULL, delimiter);
			}
		}
}



void app_main()
{
	char laengenGrad[] = "4914.13487";
    char newLaengenGrad[32];
    char breitenGrad[] = "00658.53974";
    char newBreitenGrad[32];
    char *lG;
    char *bG;

    strcpy(newLaengenGrad, roundChar(laengenGrad));
    strcpy(newBreitenGrad, roundChar(breitenGrad));

    lG = newLaengenGrad;
    lG[strlen(lG)-3] = 0;
    bG = newBreitenGrad;
    bG[strlen(bG)-3] = 0;

    setVariable("4914.135", "N", "658.540", "E");

	xTaskCreate(&echo_task, "uart_echo_task", 4096, NULL, 10, NULL);
}
*/