#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_vfs.h"
#include "esp_spiffs.h"
#include "esp_heap_caps.h"

#include "ili9340.h"
#include "fontx.h"

#include "driver/gpio.h"
#include "driver/i2s_std.h"

#include "esp_dsp.h"

static const char *TAG = "MRTS";

//Pines para LCD
#define CONFIG_WIDTH		240
#define CONFIG_HEIGHT		320
//SPI
#define CONFIG_MOSI_GPIO	23 //OK
#define CONFIG_SCLK_GPIO	18 //OK
#define CONFIG_TFT_CS_GPIO	13 //OK
#define CONFIG_DC_GPIO		27 //OK
#define CONFIG_RESET_GPIO	5  //OK
#define CONFIG_BL_GPIO		32 //OK

//Boton Back
#define BACK 16

//I2S
#define	BUFF_SIZE	320
#define SAMPLE_RATE 44100
#define FRECUENCY	100
uint16_t signal_test[BUFF_SIZE];

esp_err_t mountSPIFFS(char * path, char * label, int max_files);
static void listSPIFFS(char * path);
void paint_task(void *pvParameters);

void app_main(void){

	ESP_LOGI(TAG, "Initialize NVS");
	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
	}
	ESP_ERROR_CHECK( err );
	ESP_LOGI(TAG, "Initializing SPIFFS");
	esp_err_t ret;
	ret = mountSPIFFS("/font","storage0", 10);
	if (ret != ESP_OK) return;
	listSPIFFS("/font/");
	ret = mountSPIFFS("/waves", "storage1", 10);
	if (ret != ESP_OK) return;
	listSPIFFS("/waves/");
	ret = mountSPIFFS("/images","storage2", 14);
	if (ret != ESP_OK) return;
	listSPIFFS("/images/");

	xTaskCreate(paint_task, "ILI9341", 1024*8, NULL, 2, NULL);
}

esp_err_t mountSPIFFS(char * path, char * label, int max_files) {
	esp_vfs_spiffs_conf_t conf = {
		.base_path = path,
		.partition_label = label,
		.max_files = max_files,
		.format_if_mount_failed =true
	};

	// Use settings defined above toinitialize and mount SPIFFS filesystem.
	// Note: esp_vfs_spiffs_register is anall-in-one convenience function.
	esp_err_t ret = esp_vfs_spiffs_register(&conf);

	if (ret != ESP_OK) {
		if (ret ==ESP_FAIL) {
			ESP_LOGE(TAG, "Failed to mount or format filesystem");
		} else if (ret== ESP_ERR_NOT_FOUND) {
			ESP_LOGE(TAG, "Failed to find SPIFFS partition");
		} else {
			ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)",esp_err_to_name(ret));
		}
		return ret;
	}

	size_t total = 0, used = 0;
	ret = esp_spiffs_info(conf.partition_label, &total, &used);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG,"Failed to get SPIFFS partition information (%s)",esp_err_to_name(ret));
	} else {
		ESP_LOGI(TAG,"Mount %s to %s success", path, label);
		ESP_LOGI(TAG,"Partition size: total: %d, used: %d", total, used);
	}

	return ret;
}

static void listSPIFFS(char * path) {
	DIR* dir = opendir(path);
	assert(dir != NULL);
	while (true) {
		struct dirent*pe = readdir(dir);
		if (!pe) break;
		ESP_LOGI(__FUNCTION__,"d_name=%s", pe->d_name);
	}
	closedir(dir);
}

void paint_task(void *pvParameters){
	FontxFile fx16G[2];
	FontxFile fx24G[2];
	InitFontx(fx16G,"/font/ILGH16XB.FNT",""); // 8x16Dot Gothic
	InitFontx(fx24G,"/font/ILGH24XB.FNT",""); // 12x24Dot Gothic
	TFT_t dev;
	int up=0;

	spi_master_init(&dev,
					CONFIG_MOSI_GPIO,
					CONFIG_SCLK_GPIO,
					CONFIG_TFT_CS_GPIO,
					CONFIG_DC_GPIO,
					CONFIG_RESET_GPIO,
					CONFIG_BL_GPIO, 
					-1, 
					-1, 
					-1, 
					-1, 
					-1
				);

	lcdInit(&dev,0x9340, CONFIG_WIDTH, CONFIG_HEIGHT, 0, 0);
	lcdFillScreen(&dev, BLACK);
	lcdSetFontDirection(&dev, 1);
	lcdDrawString(&dev, fx24G, 210, 100, (uint8_t *)"TEST GRAPH", WHITE);

	gpio_set_direction(BACK, GPIO_MODE_INPUT);
	gpio_get_level(BACK);


	while(1){
		lcdDrawLine(&dev, 110,0, 110, 320, RED);
		if (gpio_get_level(BACK)==0)
		{
			lcdFillScreen(&dev, BLACK);
			lcdDrawString(&dev, fx24G, 210, 100, (uint8_t *)"BOTON ATRAS", WHITE);
			up+=50;
		
		}
		for (int i = 0; i < BUFF_SIZE; ++i)
		{
			signal_test[i]=110+70*sin(2*M_PI*i*(FRECUENCY+up)/SAMPLE_RATE);
		}
		for (int i = 1; i < BUFF_SIZE; ++i)
		{
			lcdDrawLine(&dev, signal_test[i-1],i-1, signal_test[i], i, CYAN);
		}
		if (up>=1000)
		{
			up=10;
		}
		vTaskDelay(1);
	}
}