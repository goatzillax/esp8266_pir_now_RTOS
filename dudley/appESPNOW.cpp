// Display Battery Data
// modified from Dan Geiger's code.
// by W.F.Dudley Jr.
//

#include "config.h"
#include "DudleyWatch.h"
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include "esp8266_pir_now.h"

//  since screen real estate is limited, anything received gets shoved in an assoc array, and the display cycles through what it's got.
//  problem:  no associative arrays?
//  fuck it.  i don't anticipate deleting entries.  if it fills up we just start replacing.

typedef struct {
	uint8_t mac[ESP_NOW_ETH_ALEN];
	struct_PIR_msg msg;
	unsigned long timestamp;   //  something something rollover.
} struct_ESPNOW_msg;

#define MAC_CACHE_LEN 32
struct_ESPNOW_msg mac_cache[MAC_CACHE_LEN];
int mac_cache_sz = 0;
bool plzrefresh = true;
uint8_t wifi_chan=1;
unsigned long last_update_time = 0;

//  returns -1 if not found and there's a completely free slot available.
int search_cache(const uint8_t *mac) {
	unsigned long oldest_val=0xFFFFFFFF;
	int oldest_slot = -1;

	if (mac_cache_sz == 0) {
		return(-1);
	}  // THAT WAS EASY

	for (int i=0; i<mac_cache_sz; i++) {
		if (mac_cache[i].timestamp < oldest_val) {
			oldest_slot = i;
			oldest_val = mac_cache[i].timestamp;
		}  // search for oldest slot
		if (0 == memcmp(&mac_cache[i].mac, mac, sizeof(mac_cache[i].mac))) {
			return(i);  //  lets goooooooooo
		}
	}

	//  no match was found else would ahve returned already
	if (mac_cache_sz < MAC_CACHE_LEN) {
		return(-1);
	}

	//  if cache is full, target slot is oldest slot.
	return(oldest_slot);
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
	//  probably doing way too much shit in here but Arduino
	int target_slot=0;

	if (len != sizeof(struct_PIR_msg)) {
		return;
	}
	
	target_slot = search_cache(mac);
	if (target_slot < 0) {
		target_slot = mac_cache_sz++;
	}
	memcpy(&mac_cache[target_slot].mac, mac, sizeof(mac_cache[target_slot].mac));
	memcpy(&mac_cache[target_slot].msg, incomingData, sizeof(struct_PIR_msg));
	mac_cache[target_slot].timestamp = millis();

	plzrefresh = true;
}

int display_index=0;

void refreshingDisplay() {
	last_update_time = millis();
	float batt_v;

	power->adc1Enable(AXP202_VBUS_VOL_ADC1 | AXP202_VBUS_CUR_ADC1 | AXP202_BATT_CUR_ADC1 | AXP202_BATT_VOL_ADC1, true);
	batt_v = power->getBattVoltage();

	tft->fillScreen(TFT_BLACK);
	tft->setTextColor(TFT_YELLOW, TFT_BLACK);
	tft->setTextFont(4);  // i'm an old mang, mang
	tft->setCursor(0, 0);
	tft->printf("RX ch%d sz %d", wifi_chan, mac_cache_sz);
	tft->setCursor(0, 20);

	tft->printf("%.2fv ", batt_v/1000);
	if (power->isChargeing()) {
		tft->print("+");
		tft->print(power->getBattChargeCurrent());
	}
	else {
		tft->print("-");
		tft->print(power->getBattDischargeCurrent());
	}
	tft->print("ma");

	if (mac_cache_sz == 0) {
		return;
	}  //  nothing else to display dipwad

	//  display the current index
	int cursor_y=40;
	int increment = 20;

	tft->setTextColor(TFT_GREEN, TFT_BLACK);
	tft->setCursor(0, cursor_y);
	tft->printf("%02x:%02x:%02x:%02x:%02x:%02x",	mac_cache[display_index].mac[0],
							mac_cache[display_index].mac[1],
							mac_cache[display_index].mac[2],
							mac_cache[display_index].mac[3],
							mac_cache[display_index].mac[4],
							mac_cache[display_index].mac[5]
	);

	cursor_y += increment;
	tft->setCursor(0, cursor_y);
	tft->printf("%d", mac_cache[display_index].msg.id);
	cursor_y += increment;
	tft->setCursor(0, cursor_y);
	tft->printf("%.2fv", (float) mac_cache[display_index].msg.voltage / 100);
	cursor_y += increment;
	tft->setCursor(0, cursor_y);
	tft->printf("%d", mac_cache[display_index].msg.failberts);
	cursor_y += increment;
	tft->setCursor(0, cursor_y);
	tft->printf("%.1fC", (float) mac_cache[display_index].msg.temperature / 10);
	cursor_y += increment;
	tft->setCursor(0, cursor_y);
	tft->printf("%.1f%%", (float) mac_cache[display_index].msg.humidity / 10);
	cursor_y += increment;
	tft->setCursor(0, cursor_y);
	tft->printf("%ds", (millis() - mac_cache[display_index].timestamp) / 1000);

}


void appESPNOW(void) {
	int16_t x, y;
	int mSelect;
	mac_cache_sz = 0;
	display_index = 0;
	plzrefresh = true;

	setCpuFrequencyMhz(80);
	WiFi.disconnect();
	WiFi.persistent(false);
	WiFi.mode(WIFI_STA);
	WiFi.setSleep(false);

	if (ESP_OK != esp_wifi_set_ps(WIFI_PS_NONE)) {
		Serial.println("error setting wifi ps");
		goto kthxbye;
	}

	if (ESP_OK != esp_wifi_set_promiscuous(true)) {
		Serial.println("error setting promiscuous");
		goto kthxbye;
	}
	if (ESP_OK != esp_wifi_set_channel(wifi_chan, WIFI_SECOND_CHAN_NONE)) {
		Serial.println("error setting wifi chan");
		goto kthxbye;
	}

	if (esp_now_init() != ESP_OK) {
		goto kthxbye;
	}  //  halt and go f urself

	esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
	//esp_now_register_recv_cb(reinterpret_cast<esp_now_recv_cb_t>(OnDataRecv));

	//  buncha 3-space tab nested uncommented shit, real nice.  real easy to fucking read, shitbird.
	while(1) {
		if ((millis() - last_update_time) > 2000) {
			plzrefresh = true;
		}
		if (plzrefresh) {
			plzrefresh = false;
			refreshingDisplay();  //  ahh so refresh, much wow
		}
		mSelect = poll_swipe_or_menu_press(16); // poll for touch or gesture
		switch(mSelect) {
			case LEFT:
				if (display_index > 0) {
					display_index--;
					plzrefresh = true;
				}
				break;
			case RIGHT:
				if (display_index < (mac_cache_sz-1)) {
					display_index++;
					plzrefresh = true;
				}
				break;
			case DOWN:
				if (wifi_chan < 13) {
					wifi_chan++;
					if (ESP_OK != esp_wifi_set_channel(wifi_chan, WIFI_SECOND_CHAN_NONE)) {
						Serial.println("error setting wifi chan");
						goto kthxbye2;
					}
					plzrefresh = true;
				}
				break;
			case UP:
				if (wifi_chan > 1) {
					wifi_chan--;
					if (ESP_OK != esp_wifi_set_channel(wifi_chan, WIFI_SECOND_CHAN_NONE)) {
						Serial.println("error setting wifi chan");
						goto kthxbye2;
					}
					plzrefresh = true;
				}
				break;
			case CCWCIRCLE:
				ttgo->power->shutdown();
				break;  // lol
			case CWCIRCLE:
				goto kthxbye2;
				break;
			//  no idea wtf the actual fucking physical button returns
		}
		my_idle();
	}
kthxbye2:  // wonky but whatevs
	esp_now_unregister_recv_cb();
	esp_now_deinit();
kthxbye:
	WiFi.mode(WIFI_OFF);
	setCpuFrequencyMhz(160);
	while(ttgo->getTouch(x, y)) {
		my_idle();
	}  // wait till stop touch peepee
	ttgo->bma->disableAccel();  //  yay!
	tft->fillScreen(TFT_BLACK);
}
