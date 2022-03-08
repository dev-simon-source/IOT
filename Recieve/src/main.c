#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lora.h"
#include "esp_log.h"
#include "u8g2_esp32_hal.h"
#include "driver/gpio.h"
#include "freertos/queue.h"
#include <unistd.h>

#define TAG_LORA "LoRa"
#define TAG_PROGRAM "Program"
#define TAG_DISPLAY "Display"

static const int RELAIS_PIN = 16;


void task_rx(void *p);
void setup_lora();
u8g2_t setup(u8g2_t u8g2);
void display_message(u8g2_t u8g2);
void gpio_open();
void gpio_init();

#define SDA_PIN 4
#define SCL_PIN 15
#define RST_PIN 16

uint8_t buf[8];


void task_rx(void *p)
{
	printf("Try to recieve something...");
	int x;
	for (;;)
	{
		lora_receive(); // put into receive mode
		while (lora_received())
		{
			x = lora_receive_packet(buf, sizeof(buf));
			buf[x] = 0;
			printf("Received: %s\n", buf);
			lora_receive();
			char *test = &buf;
			if (strcmp(test, "open") == 0){
				gpio_open();
			}
		}
		vTaskDelay(1);
	}
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


void gpio_open(){
	printf("Oeffne Tor\n");
	gpio_set_level(RELAIS_PIN, 0);
	sleep(1);
	gpio_set_level(RELAIS_PIN, 1);
}

void gpio_init()
{
	gpio_set_direction(RELAIS_PIN, GPIO_MODE_OUTPUT);
	gpio_set_level(RELAIS_PIN, 1);
}

void app_main()
{
	gpio_init();
	setup_lora();
	xTaskCreate(&task_rx, "task_rx", 2048, NULL, 5, NULL);
}