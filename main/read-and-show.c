#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/param.h>
#include <inttypes.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_vfs.h"
#include "esp_spiffs.h"
#include "esp_heap_caps.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include <esp_http_client.h>
#include <esp_crt_bundle.h>

#include "cJSON.h"

#include "ili9340.h"
#include "fontx.h"

#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "driver/pulse_cnt.h"

#include "esp_dsp.h"

/*****************************************************************/

#define ESP32_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define ESP32_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define ESP32_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY

#define ESP32_API_KEY        CONFIG_API_KEY
#define ESP32_CHANNEL_ID     CONFIG_CHANNEL_ID

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG_Y = "Youtube Project";

static int s_retry_num = 0;

// Buffer para acumular datos
#define MAX_BUFFER_SIZE 4096
static char buffer[MAX_BUFFER_SIZE];
static size_t buffer_len = 0;

static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static esp_err_t client_event_get_handler(esp_http_client_event_t *evt);
static void http_request_task (void *pvParameters);
void wifi_init_sta(void);

//char *subscribers;
//char *subscriber_count_str;
// Declaración global
char *subscriber_count_str = NULL;
//char *subscriber_count_str;

/*****************************************************************/

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
#define EXAMPLE_ADC_ATTEN	ADC_ATTEN_DB_0
static int adc_raw[2][10];
static int voltage[2][10];
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
void adc_task(void *pvParameters);

// Encoder Rotary
#define EC11_GPIO_A 34
#define EC11_GPIO_B 35
#define EC11_SELECT 12
#define PCNT_HIGH_LIMIT 100
#define PCNT_LOW_LIMIT  -1
int opcion=0;
int up=0;
pcnt_unit_handle_t pcnt_unit = NULL;
void pcnt_init_task(void *pvParameters);


void draw_grid(TFT_t * dev);
void draw_sin(TFT_t * dev);
void batt_status(TFT_t * dev, FontxFile *fx);

//MENU
void menu_handler();
bool updating_screen = false;
bool new_data = false;
bool menu_action = false;
bool menu = false;
bool info = true;
bool set_value  = false;
int btnok,btnbk;
uint8_t last_option=0;
int valor2=0;
enum Option {
  None,
  Autoscale,
  Vdiv,
  Sdiv,
  Offset,
  TOffset,
  Filter,
  Stop,
  Mode,
  Single,
  Clear,
  Reset,
  Probe,
  UpdateF,
  Cursor1,
  Cursor2
};
uint8_t opt = None;
uint8_t opt_value = None;
void update_screen(TFT_t * dev,FontxFile *fx);



esp_err_t mountSPIFFS(char * path, char * label, int max_files);
static void listSPIFFS(char * path);
void paint_task(void *pvParameters);

void app_main(void)
{

	ESP_LOGI(TAG, "Initialize NVS");
	esp_err_t err = nvs_flash_init();
	
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) 
	{
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
	}

/*****************************************************************/

	ESP_LOGI(TAG_Y, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

/*****************************************************************/

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

	xTaskCreate(adc_task, "BATT_STATUS", 1024, NULL, 1, NULL);
	xTaskCreate(paint_task, "ILI9341", 1024*8, NULL, 2, NULL);
	xTaskCreate(pcnt_init_task, "Encoder", 1024*4, NULL, 5, NULL);
	/*****************************************************************/

	static uint8_t ucParameterToPass;
    TaskHandle_t xHandle = NULL;
    xTaskCreate(&http_request_task, "http_request_task", 8192, &ucParameterToPass, 1, &xHandle);
    
	/*****************************************************************/
}

esp_err_t mountSPIFFS(char * path, char * label, int max_files) 
{
	esp_vfs_spiffs_conf_t conf = {
		.base_path = path,
		.partition_label = label,
		.max_files = max_files,
		.format_if_mount_failed =true
	};

	// Use settings defined above toinitialize and mount SPIFFS filesystem.
	// Note: esp_vfs_spiffs_register is anall-in-one convenience function.
	esp_err_t ret = esp_vfs_spiffs_register(&conf);

	if (ret != ESP_OK) 
	{
		if (ret ==ESP_FAIL) 
		{
			ESP_LOGE(TAG, "Failed to mount or format filesystem");
		} else if (ret== ESP_ERR_NOT_FOUND) 
		{
			ESP_LOGE(TAG, "Failed to find SPIFFS partition");
		} else 
		{
			ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)",esp_err_to_name(ret));
		}

		return ret;
	}

	size_t total = 0, used = 0;
	ret = esp_spiffs_info(conf.partition_label, &total, &used);
	
	if (ret != ESP_OK) 
	{
		ESP_LOGE(TAG,"Failed to get SPIFFS partition information (%s)",esp_err_to_name(ret));
	} else 
	{
		ESP_LOGI(TAG,"Mount %s to %s success", path, label);
		ESP_LOGI(TAG,"Partition size: total: %d, used: %d", total, used);
	}

	return ret;
}

static void listSPIFFS(char * path) 
{
	DIR* dir = opendir(path);
	assert(dir != NULL);
	while (true) 
	{
		struct dirent*pe = readdir(dir);
		if (!pe) break;
		ESP_LOGI(__FUNCTION__,"d_name=%s", pe->d_name);
	}
	
	closedir(dir);
}

void adc_task(void *pvParameters)
{
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
    while(1)
    {
    	ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &adc_raw[0][0]));
        
        if (do_calibration1_chan0) 
        {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw[0][0], &voltage[0][0]));
        }
        vTaskDelay(1);
    }
}

void pcnt_init_task(void *pvParameters)
{
	pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit = PCNT_LOW_LIMIT,
    };

    pcnt_new_unit(&unit_config, &pcnt_unit);

    pcnt_glitch_filter_config_t filter_config = {
	        .max_glitch_ns = 1000,
    };
    pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config);

    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = EC11_GPIO_A,
        .level_gpio_num = EC11_GPIO_B,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a);
    
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = EC11_GPIO_B,
        .level_gpio_num = EC11_GPIO_A,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b);

	pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
	pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
	pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
	pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    pcnt_unit_enable(pcnt_unit);
    pcnt_unit_clear_count(pcnt_unit);
    pcnt_unit_start(pcnt_unit);

    while (1) 
    {
        vTaskDelay(1);
    }
}

void paint_task(void *pvParameters)
{
	FontxFile fx16G[2];
	FontxFile fx24G[2];
	InitFontx(fx16G,"/font/ILGH16XB.FNT",""); // 8x16Dot Gothic
	InitFontx(fx24G,"/font/ILGH24XB.FNT",""); // 12x24Dot Gothic
	TFT_t dev;

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
	lcdDrawString(&dev, fx24G, 110, 15, (uint8_t *)"Cantidad de Suscriptores", WHITE);
	gpio_set_direction(BACK, GPIO_MODE_INPUT); //corregir esto para no configurarlo por cada ciclo
	gpio_set_direction(EC11_SELECT, GPIO_MODE_INPUT);


	while(1){
		menu_handler();
		//ESP_LOGW(TAG,"opcion: %d", opt);
		if (new_data || menu_action) {
      		new_data = false;
      		menu_action = false;
      		updating_screen = true;
      		update_screen(&dev,fx24G);
      		updating_screen = false;
      		vTaskDelay(pdMS_TO_TICKS(10));
    	}

		vTaskDelay(pdMS_TO_TICKS(10));
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

void draw_grid(TFT_t * dev) 
{

  for (int i = 0; i < 28; i++) {
 
  	lcdDrawPixel(dev, i * 10, 40, WHITE);
    lcdDrawPixel(dev, i * 10, 80, WHITE);
    lcdDrawPixel(dev, i * 10, 120, WHITE);
    lcdDrawPixel(dev, i * 10, 160, WHITE);
    lcdDrawPixel(dev, i * 10, 200, WHITE);
    lcdDrawPixel(dev, i * 10, 240, WHITE);
    lcdDrawPixel(dev, i * 10, 280, WHITE);

  }
  for (int i = 0; i < 320; i += 10) {
    for (int j = 0; j < 240; j += 40) {
    	lcdDrawPixel(dev, j, i, WHITE);
    }
  }
}

void draw_sin(TFT_t * dev)
{
	for (int i = 0; i < BUFF_SIZE; ++i)
	{
		signal_test[i]=110+70*sin(2*M_PI*i*(FRECUENCY+opcion*10)/SAMPLE_RATE);
	}

	for (int i = 1; i < BUFF_SIZE; ++i)
	{
		lcdDrawLine(dev, signal_test[i-1],i-1, signal_test[i], i, CYAN);
	}
	if (up>=1000)
	{
		up=10;
	}
}

void batt_status(TFT_t * dev, FontxFile *fx)
{
	uint8_t ascii[20];
	lcdDrawFillRect(dev, 220, 0, 240, 320, GRAY);
	sprintf((char *)ascii, "batt:%.2f%%", (float)(voltage[0][0]/500.0)*(100/3.7));
	lcdDrawString(dev, fx, 220, 10, ascii, WHITE);
}

void menu_handler()
{

	btnbk=gpio_get_level(BACK);
	btnok=gpio_get_level(EC11_SELECT);
	pcnt_unit_get_count(pcnt_unit, &opcion);

	if ( btnok == 0 || btnbk == 0 || (last_option != opcion))
	{
	    menu_action = true;
	}

	last_option = opcion;
}

void update_screen(TFT_t * dev,FontxFile *fx)
{
	if (!menu)
	{
		lcdFillScreen(dev, BLACK);
		//lcdDrawString(dev, fx, 0, 50, (uint8_t *)"Hello from ESP32", WHITE);
		//ESP_LOGI(TAG, "Subscriber Count: %s", subscriber_count_str);
		lcdDrawString(dev, fx, 110, 150, (uint8_t *)subscriber_count_str, WHITE);
		lcdDrawString(dev, fx, 90, 90, (uint8_t *)"Suscriptores", WHITE);
	}
	
	if (menu && set_value == false)
	{
		lcdFillScreen(dev, BLACK);
	}	
}

/*****************************************************************/

static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < ESP32_MAXIMUM_RETRY) 
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGE(TAG_Y, "retry to connect to the AP");
        } else 
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGE(TAG_Y,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) 
    {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG_Y, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static esp_err_t client_event_get_handler(esp_http_client_event_t *evt) 
{
    switch (evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGE(TAG_Y, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_DATA:
            // Verifica si hay espacio suficiente en el búfer
            if ((buffer_len + evt->data_len) < MAX_BUFFER_SIZE) 
            {
                // Copia los datos al búfer
                memcpy(buffer + buffer_len, evt->data, evt->data_len);
                buffer_len += evt->data_len;
            }
            break;
        case HTTP_EVENT_ON_FINISH:
            //printf("Solicitud HTTP completada. Totaconst char *subscriber_count_str = subscriber_count->valuestring;const char *subscriber_count_str = subscriber_count->valuestring;const char *subscriber_count_str = subscriber_count->valuestring;l de bytes recibidos: %zu\n", buffer_len);

            cJSON *json_root = cJSON_Parse(buffer);
            if (json_root != NULL) 
            {

                cJSON *items = cJSON_GetObjectItemCaseSensitive(json_root, "items");
                cJSON *first_item = cJSON_GetArrayItem(items, 0);
                cJSON *statistics = cJSON_GetObjectItemCaseSensitive(first_item, "statistics");
                cJSON *subscriber_count = cJSON_GetObjectItemCaseSensitive(statistics, "subscriberCount");

                if (cJSON_IsString(subscriber_count)) 
                {
                    //const char *subscriber_count_str = subscriber_count->valuestring;
					subscriber_count_str = strdup(subscriber_count->valuestring);
                    ESP_LOGI(TAG_Y, "Subscriber Count: %s", subscriber_count_str);
                }

                cJSON_Delete(json_root);
            } else 
            {
                ESP_LOGE(TAG_Y, "Error al analizar JSON.\n");
            }

            buffer_len = 0; // Se limpia el buffer
            break;
        default:
            break;
    }
    return ESP_OK;
}

static void http_request_task (void *pvParameters)
{
    while (1) {

        // Se configura la petición HTTP
        esp_http_client_config_t config_get = {
            //.url = "https://www.googleapis.com/youtube/v3/channels?id="ESP32_CHANNEL_ID"&key="ESP32_API_KEY"&part=snippet,statistics",
            .url = "https://www.googleapis.com/youtube/v3/channels?id="ESP32_CHANNEL_ID"&key="ESP32_API_KEY"&part=statistics",
            .method = HTTP_METHOD_GET,
            .cert_pem = NULL,
            .event_handler = client_event_get_handler,
            .crt_bundle_attach = esp_crt_bundle_attach,
        };

        esp_http_client_handle_t client = esp_http_client_init(&config_get);

        // Realiza la solicitud HTTP
        esp_err_t err = esp_http_client_perform(client);

        if (err == ESP_OK) 
        {
            ESP_LOGI(TAG_Y, "Solicitud HTTP exitosa\n");
        } else 
        {
            ESP_LOGE(TAG_Y, "Error en la solicitud HTTP: %d\n", err);
        }

        // Limpia el cliente HTTP
        esp_http_client_cleanup(client);

        // Reinicia en 1 segundo
        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ESP32_WIFI_SSID,
            .password = ESP32_WIFI_PASS
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG_Y, "wifi_init_sta finished.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG_Y, "connected to ap SSID:%s password:%s", ESP32_WIFI_SSID, ESP32_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG_Y, "Failed to connect to SSID:%s, password:%s", ESP32_WIFI_SSID, ESP32_WIFI_PASS);
    } else {
        ESP_LOGE(TAG_Y, "UNEXPECTED EVENT");
    }
}

/*****************************************************************/