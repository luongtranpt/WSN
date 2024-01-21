/**
 * @file wifi_sta.h
 * @author Vanperdung (dung.nv382001@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-02-09
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef _WIFI_STA_H_
#define _WIFI_STA_H_

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

void wifi_init(void);
void wifi_sta(wifi_config_t wifi_cfg, wifi_mode_t wifi_mode);
#endif