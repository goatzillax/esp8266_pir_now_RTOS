#ifndef ESP8266_PIR_NOW_H
#define ESP8266_PIR_NOW_H

typedef struct {
   uint16_t	id;
   int16_t	voltage;	//  voltage * 100
   uint32_t	failberts;
   int16_t	temperature;	//  temperature(c) * 10
   int16_t	humidity;	//  humidity * 10
} struct_PIR_msg;

#endif
