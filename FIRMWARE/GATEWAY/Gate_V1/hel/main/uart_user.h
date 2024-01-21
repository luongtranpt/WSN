/**
 * @file uart_user.h
 * @author Vanperdung (dung.nv382001@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-02-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef _UART_USER_H_
#define _UART_USER_H_

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#define UART1_TXD_PIN GPIO_NUM_17 
#define UART1_RXD_PIN GPIO_NUM_16

typedef struct 
{
    char action[16];
    int node_id;
    int slot_id;
    int period;
    double threshold;
} uart_obj_t;

void uart_user_task(void *param);

#endif