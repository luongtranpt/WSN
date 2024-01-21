/**
 * @file button.h
 * @author Vanperdung (dung.nv382001@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-02-09
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef _BUTTON_H_
#define _BUTTON_H_

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#define TIME_HOLD (3000 / portTICK_RATE_MS)
#define TIME_CLICK_MIN (20 / portTICK_RATE_MS)
#define TIME_CLICK_MAX (1000 / portTICK_RATE_MS)
#define TIME_RESET (1000 / portTICK_RATE_MS)

void button_task(void *param);

#endif