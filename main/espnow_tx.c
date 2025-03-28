#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/adc.h"
#include "esp_log.h"
#include "tcpip_adapter.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_sleep.h"
#include "esp8266_pir_now.h"
//#include "secrets.h"

static const char *TAG = "espnow_tx";

float voltage = 0;

static TaskHandle_t mainTask = NULL;

static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

enum e_xmit_state {
   XMIT_IDLE,
   XMIT_STARTED,
   XMIT_FINISHED
};


void setup_adc() {
	adc_config_t adc_config;

	adc_config.mode = ADC_READ_TOUT_MODE;
	//  Do not know the relevance or tradeoffs of this one.  nor the blurb about vdd33_const.
	adc_config.clk_div = 8;
	ESP_ERROR_CHECK(adc_init(&adc_config));
}


void run_adc() {
	uint16_t adc_data;
	uint32_t sum=0;
	setup_adc();

	//  fast version sez need interrupt disable but I no see that
	for (int i=0; i<CONFIG_ADC_SAMPLECNT; i++) {
		if (ESP_OK != adc_read_nofuckup(&adc_data)) {
			ESP_LOGE(TAG, "adc read %d failed\n", i);
		}
		sum += adc_data; 
	}

	adc_data = (uint16_t) (sum / CONFIG_ADC_SAMPLECNT);
//	voltage = ((float) adc_data / 1023.0f) * (220.0f + 42.0f + 100.0f) / 100.0f;
	voltage = ((float) adc_data / 1023.0f) * (220.0f + 1200.0f + 100.0f) / 100.0f;
	ESP_LOGI(TAG, "ADC Value %d, %.2f v\n", adc_data, voltage);  // float print requires disable nano formatting of newlib component
}


static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
	//  this is supposed to be running "from the wifi task".  docs say it's a high priority task but never mentions if it's ISR.
	xTaskNotifyGive(mainTask);
	//  Uh...  error checking?
} 

#if CONFIG_STATION_MODE
#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA
#else
#define ESPNOW_WIFI_MODE WIFI_MODE_AP
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_AP
#endif

esp_now_peer_info_t *peer;  //  ugh.  lost littlefs and rtcmem code, otherwise would use this to scan.

void setup_espnow() {
	tcpip_adapter_init();

	ESP_ERROR_CHECK(esp_event_loop_create_default());

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
	ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
	ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );
	ESP_ERROR_CHECK( esp_wifi_start());

	ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, 2) );

	ESP_ERROR_CHECK( esp_now_init() );
	ESP_ERROR_CHECK( esp_now_register_send_cb(espnow_send_cb) );

	peer = malloc(sizeof(esp_now_peer_info_t));
	memset(peer, 0, sizeof(esp_now_peer_info_t));
	peer->channel = CONFIG_ESPNOW_CHANNEL;
	peer->ifidx = ESPNOW_WIFI_IF;
	peer->encrypt = false;
	memcpy(peer->peer_addr, broadcast_mac, ESP_NOW_ETH_ALEN);
	ESP_ERROR_CHECK( esp_now_add_peer(peer) );

#ifdef USE_UNICAST
	memcpy(peer->peer_addr, unicast_mac, ESP_NOW_ETH_ALEN);
	ESP_ERROR_CHECK( esp_now_add_peer(peer) );
#endif

}

struct_PIR_msg msg;  //  dunno what the lifetime of this should be, but we passed the address.

void run_espnow() {
	setup_espnow();

	msg.id=0;
	msg.voltage=voltage * 100;
	msg.failberts=0;
	msg.temperature=1000;
	msg.humidity=1500;

#ifdef USE_UNICAST
	if (esp_now_send(unicast_mac, (uint8_t *) &msg, sizeof(msg)) != ESP_OK) {
#else
	if (esp_now_send(broadcast_mac, (uint8_t *) &msg, sizeof(msg)) != ESP_OK) {
#endif
		ESP_LOGE(TAG, "Send error");
	}
}


void app_main() {
	mainTask = xTaskGetCurrentTaskHandle();

	run_adc();
	run_espnow();

	while (!ulTaskNotifyTake(pdFALSE, portMAX_DELAY));

	//  can has goto here on error?  maintask -> goto error, any other task -> immediate notify?
//err:
	esp_wifi_stop();
	fflush(stdout);
	esp_deep_sleep(30*1000000);
}
