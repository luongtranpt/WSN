/**
 * @file mqtt.h
 * @author Vanperdung (dung.nv382001@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-02-09
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef _MQTT_H_
#define _MQTT_H_

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "Queue.h"

#define MQTT_BROKER "broker.hivemq.com"


RingbufHandle_t mqtt_ring_buf;
esp_mqtt_client_handle_t client;
char Handler[50];
uint8_t flagmqtt;
queue_t xQueue[10];


#define USERNAME "vpnh1tk356qk657zwhtk"


#define TOPIC "v1/devices/me/telemetry"
#define Server "mqtt://demo.thingsboard.io"

void mqtt_client_sta(void);

#endif