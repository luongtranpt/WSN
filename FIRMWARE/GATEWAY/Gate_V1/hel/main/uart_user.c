/**
 * @file uart_user.c
 * @author Vanperdung (dung.nv382001@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-02-07
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"

#include "nvs_flash.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_smartconfig.h"
#include "mqtt_client.h"
#include "esp_spiffs.h"
#include "esp_err.h"
#include "esp_attr.h"
#include "esp_http_server.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/spi_master.h"

#include "cJSON.h"
#include "sx1278.h"
#include "uart_user.h"

static const char *TAG = "UART USER";
extern sx1278_network_t sx1278_network;
extern sx1278_attr_cfg_t attr_cfg_temp;

void uart_init(void)
{
	uart_config_t uart_config = {
			.baud_rate 	= 115200,
			.data_bits 	= UART_DATA_8_BITS,
			.parity 	= UART_PARITY_DISABLE,
			.stop_bits 	= UART_STOP_BITS_1,
			.flow_ctrl 	= UART_HW_FLOWCTRL_DISABLE,
			.source_clk = UART_SCLK_APB,
	};

	uart_driver_install(UART_NUM_0, 256 * 2, 0, 0, NULL, 0);
	uart_param_config(UART_NUM_0, &uart_config);
	uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

esp_err_t uart_check_action(uart_obj_t uart_obj)
{
    if (strcmp(uart_obj.action, "add_node") == 0)
    {
        if (sx1278_network.node_slots[uart_obj.slot_id].node_id != 0)
        {
            ESP_LOGE(TAG, "ERROR: slot_id is used");
            return ESP_FAIL;
        }
        for (int i = 0; i < NW_DEFAULT_TOTAL_SLOTS; i++)
        {
            if (sx1278_network.node_slots[i].node_id == (uint16_t)uart_obj.node_id)
            {
                ESP_LOGI(TAG, "ERROR: node_id existed");
                return ESP_FAIL;
            }
        }
        return ESP_OK;
    }
    else if (strcmp(uart_obj.action, "cfg_period") == 0)
    {
        if (uart_obj.period < 5)
        {
            ESP_LOGI(TAG, "ERROR: Period need longer than 5s");
            return ESP_FAIL;
        }
        else
            return ESP_OK;
    }
    else
        return ESP_FAIL;
}

esp_err_t uart_parse_data(char *uart_data, uart_obj_t *uart_obj)
{
    cJSON *root = cJSON_Parse(uart_data);
    if (root == NULL)
        return ESP_FAIL;
    cJSON *cur_elem = NULL;
    cJSON_ArrayForEach(cur_elem, root)
    {
        if (cur_elem->string)
        {
            const char *cur_str = cur_elem->string;
            if (strcmp(cur_str, "action") == 0)
                memcpy(uart_obj->action, cur_elem->valuestring, strlen(cur_elem->valuestring) + 1);
            else if (strcmp(cur_str, "node_id") == 0)
                uart_obj->node_id = cur_elem->valueint;
            else if (strcmp(cur_str, "slot_id") == 0)
                uart_obj->slot_id = cur_elem->valueint;
            else if (strcmp(cur_str, "period") == 0)
                uart_obj->period = cur_elem->valueint;
            else if (strcmp(cur_str, "threshold") == 0)
                uart_obj->threshold = cur_elem->valuedouble;
        }
    }
    cJSON_Delete(root);
    return ESP_OK;
}

void uart_user_task(void *param)
{
    uint8_t data_recv[256] = {0};
    uart_obj_t uart_obj;
    uart_init();
    while (1)
    {
        int rxBytes = uart_read_bytes(UART_NUM_0, data_recv, 256, 50 / portTICK_RATE_MS);
        if (rxBytes > 0)
        {
            data_recv[rxBytes] = '\0';
            ESP_ERROR_CHECK(uart_parse_data((char *)data_recv, &uart_obj));
            if (strcmp(uart_obj.action, "add_node") == 0)
            {
                if (uart_check_action(uart_obj) == ESP_OK)
                {
                    ESP_LOGI(TAG, "NODE_ID: %d, SLOT_ID: %d, THRESHOLD: %f", uart_obj.node_id, uart_obj.slot_id, (float)uart_obj.threshold);
                    sx1278_network.node_slots[uart_obj.slot_id].node_id = uart_obj.node_id;
                    attr_cfg_temp.threshold[uart_obj.slot_id] = (float)uart_obj.threshold;
                }
            }
            else if (strcmp(uart_obj.action, "cfg_period") == 0)
            {
                if (uart_check_action(uart_obj) == ESP_OK)
                {
                    ESP_LOGI(TAG, "PERIOD: %d", uart_obj.period);
                    attr_cfg_temp.period = uart_obj.period;
                }
            }
            else if (strcmp(uart_obj.action, "start") == 0)
            {
                ESP_LOGI(TAG, "START"); 
                sx1278_network.flags.network_run = true;
            }
            else if (strcmp(uart_obj.action, "end") == 0)
            {
                ESP_LOGI(TAG, "END"); 
                sx1278_network.flags.network_run = false;
            }
            else
                ESP_LOGE(TAG, "Cannot find action");
        }
    }
}