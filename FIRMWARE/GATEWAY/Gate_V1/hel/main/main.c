/**
 * @file main.c
 * @author Vanperdung (dung.nv382001@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-01-31
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
#include "esp_http_client.h"
#include "esp_attr.h"
#include "esp_http_server.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/spi_master.h"

#include "sx1278.h"
#include "uart_user.h"
#include "button.h"
#include "common.h"
#include "led.h"
#include "mqtt.h"
#include "smartcfg.h"
#include "wifi_sta.h"
#include "wifi_ap.h"
#include "webserver.h"

static const char *TAG = "MAIN";
gateway_mode_t mode = LOCAL;
gateway_cfg_mode_t cfg_mode = NOT_STATE;
RTC_NOINIT_ATTR int gateway_mode_flag;
httpd_handle_t server;
RingbufHandle_t webserver_ring_buf;

void app_main(void)
{
   esp_err_t err;

    ESP_LOGI(TAG, "Initializing...");

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    // wifi_init();
    // if (gateway_mode_flag == SMARTCONFIG_MODE)
    // {
    //     gateway_mode_flag = NORMAL_MODE;
    //     mode = CONFIG;
    //     cfg_mode = SMARTCONFIG;
    //     smartconfig_init();
    // }
    // else if (gateway_mode_flag == WIFI_SOFTAP_MODE)
    // {
    // }
    // else
    // {
    //     wifi_config_t wifi_cfg = {
    //         .sta = {
    //             .threshold.authmode = WIFI_AUTH_WPA2_PSK,
    //             .pmf_cfg = {
    //                 .capable = true,
    //                 .required = false,
    //             },
    //                 .ssid = "TL",
    //                 .password = "11111111",
    //         },
    //     };
    //     if (esp_wifi_get_config(ESP_IF_WIFI_STA, &wifi_cfg) == ESP_OK)
    //     {
    //         ESP_LOGI(TAG, "Wifi configuration already stored in flash partition called NVS");
    //         ESP_LOGI(TAG, "%s", wifi_cfg.sta.ssid);
    //         ESP_LOGI(TAG, "%s", wifi_cfg.sta.password);
    //         wifi_sta(wifi_cfg, WIFI_MODE_STA);
    //         mqtt_client_sta();
            xTaskCreate(&sx1278_task, "SX1278", 4096, NULL, 5, NULL);
//             xTaskCreate(&sx1279_task, "SX1278", 4096, NULL, 4, NULL);
//            xTaskCreate(&uart_user_task, "UART USER", 4096, NULL, 6, NULL);
//         }
//    }
}