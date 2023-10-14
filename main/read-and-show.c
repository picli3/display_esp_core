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
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

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

// Battery sense
#define EXAMPLE_ADC1_CHAN0	ADC_CHANNEL_0
#define EXAMPLE_ADC_ATTEN	ADC_ATTEN_DB_11
static int adc_raw[2][10];
static int voltage[2][10];
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);


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
	uint8_t ascii[20];

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


	//-------------ADC1 Init---------------//
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
 	ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

  	//-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = EXAMPLE_ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN0, &config));

    //-------------ADC1 Calibration Init---------------//
    adc_cali_handle_t adc1_cali_chan0_handle = NULL;
    bool do_calibration1_chan0 = example_adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC1_CHAN0, EXAMPLE_ADC_ATTEN, &adc1_cali_chan0_handle);

	while(1){
		ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &adc_raw[0][0]));
        ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN0, adc_raw[0][0]);
        if (do_calibration1_chan0) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw[0][0], &voltage[0][0]));
            ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %.2f V", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN0, (float)voltage[0][0]/500.0);
       		lcdFillScreen(&dev, BLACK);
       		sprintf((char *)ascii, "batt: %.2f V", (float)voltage[0][0]/500.0);
       		lcdDrawString(&dev, fx24G, 210, 10, ascii, WHITE);
        }

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

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}