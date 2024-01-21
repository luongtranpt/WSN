/**
 * @file wifi_ap.h
 * @author Vanperdung (dung.nv382001@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-02-09
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef _WIFI_AP_H_
#define _WIFI_AP_H_

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#define WIFI_AP_SSID "GATEWAY_LORA"
#define WIFI_AP_CHANNEL 1
#define WIFI_AP_MAX_CONN 1

void wifi_init_softap(void);

#endif