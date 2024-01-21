/**
 * @file mqtt.c
 * @author Vanperdung (dung.nv382001@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-02-09
 *
 * @copyright Copyright (c) 2023
 *
 */

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

extern gateway_mode_t mode;
extern gateway_cfg_mode_t cfg_mode;
static const char *TAG = "MQTT";

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    client = event->client;
     int msg_id;
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
    {
       // mode = NORMAL;
       esp_mqtt_client_subscribe(client, "v1/devices/me/attributes", 1 );
       esp_mqtt_client_subscribe(client, "v1/devices/me/rpc/request/+", 1 );
       ESP_LOGI(TAG, "MQTT event connected");
        break;
    }
    case MQTT_EVENT_DISCONNECTED:
        mode = LOCAL;
        ESP_LOGI(TAG, "MQTT event disconnected");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT event subcribed, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT event unsubcribed, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT event published, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
    {
        memset(Handler, '\0', sizeof(Handler));
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        memcpy(Handler,event->data,event->data_len);
        ESP_LOGI(TAG,"DATA: %s",Handler);
        int index = 0 ;
        char str[11];
        for( int i = 11 ; i <= 21; i++ )
        {
            str[index] = Handler[i];
            index++;
        }
        ESP_LOGI("HELLO","%s",str);
        if (strcmp(str,"accept_node") == 0)
        {
              flagmqtt = 1; 
        }
        
      
        break;
    }
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT event error");
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static void mqtt_task(void *param)
{
    char *mess_recv = NULL;
    size_t mess_size = 0;
    mqtt_obj_t mqtt_obj;
    while (1)
    {
        mess_recv = (char *)xRingbufferReceive(mqtt_ring_buf, &mess_size, portMAX_DELAY);
        if (mess_recv)
        {
            mess_recv[mess_size] = '\0';
            ESP_LOGI(TAG, "Recv payload: %s", mess_recv);
            memset(&mqtt_obj, 0, sizeof(mqtt_obj));
            mqtt_parse_data(mess_recv, &mqtt_obj);

            vRingbufferReturnItem(mqtt_ring_buf, (void *)mess_recv);
        }
    }
}

void mqtt_client_sta(void)
{
    uint8_t broker[50] = {0};
    ESP_LOGI(TAG, "MQTT init");
    // ESP_LOGI(TAG, "Broker: %s", MQTT_BROKER);
    // sprintf((char *)broker, "mqtt://%s", MQTT_BROKER);
    // mqtt_ring_buf = xRingbufferCreate(4096, RINGBUF_TYPE_NOSPLIT);
    // if (mqtt_ring_buf == NULL)
    //     ESP_LOGE(TAG, "Failed to create ring buffer");
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = Server,
    //    .keepalive = 60,
        .username = USERNAME,
    };
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
   // xTaskCreate(&mqtt_task, "mqtt_task", 4096, NULL, 9, NULL);
}
