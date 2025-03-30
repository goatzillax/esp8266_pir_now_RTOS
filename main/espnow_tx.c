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
#include "esp_attr.h"
#include "esp8266_pir_now.h"
//#include "secrets.h"

static const char *TAG = "espnow_tx";

float voltage = 0;

static TaskHandle_t mainTask = NULL;

static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

RTC_DATA_ATTR uint32_t wifi_channel=CONFIG_ESPNOW_CHANNEL;  //  wifi-chan start chan at zero chan desu.  chan == 0 means scan.
RTC_DATA_ATTR uint32_t failberts=0;

enum e_xmit_state {
   XMIT_IDLE,
   XMIT_STARTED,
   XMIT_FINISHED
};


void go2sleep() {
	esp_wifi_stop();
	fflush(stdout);
	esp_deep_sleep(CONFIG_DEEPSLEEP_TIME);
}


void setup_adc() {
	adc_config_t adc_config;

	adc_config.mode = ADC_READ_TOUT_MODE;
	//  Do not know the relevance or tradeoffs of this one.  nor the blurb about vdd33_const.
	adc_config.clk_div = 8;
	ESP_ERROR_CHECK(adc_init(&adc_config));
}

uint16_t adc_data;

void run_adc() {
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
	float aux_resistor = (float) CONFIG_ADC_AUX_RESISTANCE;
	voltage = ((float) adc_data / 1023.0f) * (aux_resistor + 220.0f + 100.0f) / 100.0f;
	ESP_LOGI(TAG, "ADC Value %d, %.2f v\n", adc_data, voltage);  // float print requires disable nano formatting of newlib component
}

static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
	//  this is supposed to be running "from the wifi task".  docs say it's a high priority task but never mentions if it's ISR.
	xTaskNotifyGive(mainTask);

	if (ESP_NOW_SEND_SUCCESS != status) {
		failberts++;
	}
	else {
		failberts=0;
	}
} 

#if CONFIG_STATION_MODE
#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA
#else
#define ESPNOW_WIFI_MODE WIFI_MODE_AP
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_AP
#endif

esp_now_peer_info_t *peer;
struct_PIR_msg msg;  //  dunno what the lifetime of this should be, but we passed the address.

int scanned=0;

void send_message() {
#ifdef USE_UNICAST
	if (esp_now_send(unicast_mac, (uint8_t *) &msg, sizeof(msg)) != ESP_OK) {
#else
	if (esp_now_send(broadcast_mac, (uint8_t *) &msg, sizeof(msg)) != ESP_OK) {
#endif
		ESP_LOGE(TAG, "Send error");
		go2sleep();  //  fuck it
	}
}


int send_message_blocking(int tries, int delay) {
	for (int i=0; i<tries; i++) {
		send_message();
		while (!ulTaskNotifyTake(pdFALSE, portMAX_DELAY));

		if (failberts == 0) {
			return(1);
		}

		vTaskDelay(delay / portTICK_PERIOD_MS);
	}
	return(0);
}


void setup_espnow() {
	tcpip_adapter_init();

	ESP_ERROR_CHECK(esp_event_loop_create_default());

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
	ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
	ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );
	ESP_ERROR_CHECK( esp_wifi_start());

	ESP_ERROR_CHECK( esp_now_init() );
	ESP_ERROR_CHECK( esp_now_register_send_cb(espnow_send_cb) );

	peer = malloc(sizeof(esp_now_peer_info_t));
	memset(peer, 0, sizeof(esp_now_peer_info_t));
	peer->ifidx = ESPNOW_WIFI_IF;
	peer->encrypt = false;

	if (wifi_channel == 0) {
#ifdef USE_UNICAST
		ESP_LOGI(TAG, "Running scan\n");
		//  mandatory 3 tries for scan
		for (int i=0; i<3; i++) {
			int scan_chan = 13;  //  seems like we should ping pong these in a sequence to avoid schmear?
			while (scan_chan > 0) {
				ESP_ERROR_CHECK( esp_wifi_set_channel(scan_chan, scan_chan) );
				peer->channel = scan_chan;  // this feels useless to me
				memcpy(peer->peer_addr, unicast_mac, ESP_NOW_ETH_ALEN);
				ESP_ERROR_CHECK( esp_now_add_peer(peer) );

				msg.id = scan_chan;
				msg.voltage=voltage * 100;
				msg.failberts=failberts;
				msg.temperature=1000;
				msg.humidity=1500;

				//  making too many tries seems to hose results due to schmear
				if (send_message_blocking(1, 100)) {
					ESP_LOGI(TAG, "selected channel %d\n", scan_chan);
					wifi_channel = scan_chan;
					scanned = 1;
					break;
				}  // means it succeeded
				else {
					ESP_ERROR_CHECK( esp_now_del_peer(unicast_mac) );  // might could also use the mod
				}
				scan_chan--;
			}
			if (wifi_channel != 0) {
				break;
			}  //  success
		}

		//  if wifi_channel still boned need to restart.
		if (wifi_channel == 0) {
			go2sleep();
		}
#else
		ESP_LOGE(TAG, "Requested scan without unicast\n");
		go2sleep();
#endif
	}  //  need to initiate scan
	else {
		ESP_ERROR_CHECK( esp_wifi_set_channel(wifi_channel, wifi_channel) );
#ifdef USE_UNICAST
		memcpy(peer->peer_addr, unicast_mac, ESP_NOW_ETH_ALEN);
		ESP_ERROR_CHECK( esp_now_add_peer(peer) );
#endif
	}
	//  add in the broadcast channel just in case I guess
	peer->channel = wifi_channel;  // this feels useless to me
	memcpy(peer->peer_addr, broadcast_mac, ESP_NOW_ETH_ALEN);
	ESP_ERROR_CHECK( esp_now_add_peer(peer) );

}


void run_espnow() {
	setup_espnow();

	msg.id=adc_data;
	msg.voltage=voltage * 100;
	msg.failberts=failberts;
	msg.temperature=1000;
	msg.humidity=1500;

	//  don't send another one if scan was run
	if (scanned == 0) {
		send_message_blocking(CONFIG_ESPNOW_TRIES, CONFIG_ESPNOW_SEND_DELAY);
	}
}


void app_main() {
	//vTaskDelay(5000 / portTICK_PERIOD_MS);
	mainTask = xTaskGetCurrentTaskHandle();

	//  ok so there is something hinkey going on with the logging faciliy, almost like this resolves way later than it should.
	ESP_LOGI(TAG, "wifi channel %d\n", wifi_channel);
	ESP_LOGI(TAG, "failberts %d\n", failberts);

	run_adc();
	run_espnow();

	//  can has goto here on error?  maintask -> goto error, any other task -> immediate notify?
//err:
	go2sleep();
}
