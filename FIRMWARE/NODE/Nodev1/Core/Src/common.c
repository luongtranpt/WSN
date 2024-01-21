/*
 * common.c
 *
 *  Created on: Feb 8, 2023
 *      Author: dung
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/unistd.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <time.h>

#include "main.h"

#include "common.h"
// #include "usbd_cdc.h"

#ifdef USE_USB
#include "usbd_cdc_if.h"
extern USBD_HandleTypeDef hUsbDeviceFS;
#endif





#if DEBUG == 1

static char string[300];

void LOG(const char *TAG, char *data)
{
	char data_log[100] = {0};
	sprintf(data_log, "%s: %s\n", TAG, data);
	// HAL_UART_Transmit(&huart1, (uint8_t*)data_log, strlen(data_log), 1000);
#ifdef USE_USB
    CDC_Transmit_FS(data_log, strlen(data_log));
#endif /* USE_USB */
}

/**
 * @brief 
 * 
 * @param fmt 
 * @param argp 
 */
void vprint(const char *fmt, va_list argp)
{
    memset(string, '\0', sizeof(string));
    if(0 < vsprintf(string, fmt, argp)) // build string
    {
        // HAL_UART_Transmit(UART_DEBUG, (uint8_t*)string, strlen(string), 100); // send message via UART
#ifdef USE_USB
        CDC_Transmit_FS(string, strlen(string));
#endif /* USE_USB */
    }
}

/**
 * @brief custom printf() function
 * 
 * @param fmt 
 * @param ... 
 */
void logPC(const char *fmt, ...)
{
    va_list argp;
    va_start(argp, fmt);
    vprint(fmt, argp);
    va_end(argp);
}

#else

void LOG(const char *TAG, char *data)
{
    UNUSED(TAG);
    UNUSED(data);
}

void logPC(const char *fmt, ...)
{
    UNUSED(fmt);
}
#endif






void reverse(char* str, int len)
{
    int i = 0, j = len - 1, temp;
    while (i < j)
    {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}

int intToStr(int x, char str[], int d)
{
    int i = 0;
    if (x == 0)
    {
        str[i++] = '0';
    }
    else
    {
        while (x)
        {
            str[i++] = (x % 10) + '0';
            x = x / 10;
        }
    }
    
    // If number of digits required is more, then
    // add space at the beginning
    while (i < d)
        str[i++] = ' ';

    reverse(str, i);
    str[i] = '\0';
    return i;
}

void ftoa(float n, char* res, int afterpoint)
{
    // Extract integer part
    int ipart = (int)n;
    // Extract floating part
    float fpart = n - (float)ipart;
    // convert integer part to string
    int i = intToStr(ipart, res, 0);
    // check for display option after point
    if (afterpoint != 0)
    {
        res[i] = '.'; // add dot
        fpart = fpart * pow(10, afterpoint);
        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}

int intToStr0(int x, char str[], int d)
{
    int i = 0;
    if (x == 0)
    {
        str[i++] = '0';
    }
    else
    {
        while (x)
        {
            str[i++] = (x % 10) + '0';
            x = x / 10;
        }
    }
    
    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';
    return i;
}

void ftoa0(float n, char* res, int afterpoint)
{
    // Extract integer part
    int ipart = (int)n;
    // Extract floating part
    float fpart = n - (float)ipart;
    // convert integer part to string
    int i = intToStr0(ipart, res, 0);
    // check for display option after point
    if (afterpoint != 0)
    {
        res[i] = '.'; // add dot
        fpart = fpart * pow(10, afterpoint);
        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}

int random_number(int min_num, int max_num)
{
    int result = 0, low_num = 0, hi_num = 0;

    if (min_num < max_num)
    {
        low_num = min_num;
        hi_num = max_num + 1; // include max_num in output
    } else {
        low_num = max_num + 1; // include max_num in output
        hi_num = min_num;
    }

    // srand(HAL_GetTick());
    result = (rand() % (hi_num - low_num)) + low_num;
    return result;
}

