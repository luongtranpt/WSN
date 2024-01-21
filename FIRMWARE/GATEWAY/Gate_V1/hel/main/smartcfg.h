/**
 * @file smartcfg.h
 * @author Vanperdung (dung.nv382001@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-02-09
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef _SMARTCFG_H_
#define _SMARTCFG_H_

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#define CONNECTED_BIT       BIT0
#define ESPTOUCH_DONE_BIT   BIT1

void smartconfig_init(void);

#endif