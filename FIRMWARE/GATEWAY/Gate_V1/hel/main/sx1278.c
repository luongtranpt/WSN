/**
 * @file sx1278.c
 * @author Vanperdung (dung.nv382001@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-01-29
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
#include "esp_attr.h"
#include "esp_http_server.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/spi_master.h"

#include "sx1278.h"
#include "uart_user.h"
#include "stdio.h"
#include "string.h"
#include "mqtt.h"
#include "Queue.h"

static const char *TAG = "SX1278_GATEWAY";
static spi_device_handle_t spi_handle;
EventGroupHandle_t sx1278_evt_group;

// sx1278_network_t sx1278_network = {0};
// sx1278_attr_cfg_t attr_cfg_temp = {0};

void reverse(char *str, int len)
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
        str[i++] = '0';

    while (x)
    {
        str[i++] = (x % 10) + '0';
        x = x / 10;
    }

    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';
    return i;
}

void ftoa(double n, char *res, int afterpoint)
{
    int ipart = (int)n;
    double fpart = n - (double)ipart;
    int i = intToStr(ipart, res, 0);
    if (afterpoint != 0)
    {
        res[i] = '.';
        fpart = fpart * pow(10, afterpoint);
        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}

void IRAM_ATTR sx1278_intr_handler(void *arg)
{
    gpio_num_t pin = (gpio_num_t)arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (pin == SX1278_DIO0_PIN)
        xEventGroupSetBitsFromISR(sx1278_evt_group, SX1278_DIO0_BIT, &xHigherPriorityTaskWoken);
    // else if (pin == SX1278_DIO3_PIN)
    //     xEventGroupSetBitsFromISR(sx1278_evt_group, SX1278_DIO3_BIT, &xHigherPriorityTaskWoken);
    // else if (pin == SX1278_DIO4_PIN)
    //     xEventGroupSetBitsFromISR(sx1278_evt_group, SX1278_DIO4_BIT, &xHigherPriorityTaskWoken);
}

void sx1278_gpio_init(void)
{
    gpio_config_t io0_config = {
        .pin_bit_mask = BIT64(SX1278_DIO0_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLDOWN_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    gpio_config(&io0_config);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(SX1278_DIO0_PIN, sx1278_intr_handler, (void *)SX1278_DIO0_PIN);
}

void sx1278_spi_init(void)
{
    spi_bus_config_t bus_config = {
        .mosi_io_num = SX1278_MOSI_PIN,
        .miso_io_num = SX1278_MISO_PIN,
        .sclk_io_num = SX1278_SCK_PIN,
        .max_transfer_sz = 0,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    assert(spi_bus_initialize(VSPI_HOST, &bus_config, 0) == ESP_OK);

    spi_device_interface_config_t device_config = {
        .clock_speed_hz = 4000000,
        .mode = 0,
        .spics_io_num = SX1278_NSS_PIN,
        .queue_size = 1,
        .pre_cb = NULL,
        .flags = 0,
    };
    assert(spi_bus_add_device(VSPI_HOST, &device_config, &spi_handle) == ESP_OK);
}

uint8_t sx1278_read_reg(uint8_t reg)
{
    uint8_t data_send[2] = {0x00 | reg, 0xFF}; // A wnr bit, which is 1 for write and 0 for read access
    uint8_t data_recv[2] = {0};
    spi_transaction_t sx1278_recv = {
        .flags = 0,
        .length = 8 * sizeof(data_send),
        .rx_buffer = (void *)data_recv,
        .tx_buffer = (void *)data_send,
    };
    gpio_set_level(SX1278_NSS_PIN, 0);
    spi_device_transmit(spi_handle, &sx1278_recv);
    gpio_set_level(SX1278_NSS_PIN, 1);
    return data_recv[1];
}

void sx1278_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t data_send[2] = {0x80 | reg, val}; // A wnr bit, which is 1 for write and 0 for read access
    uint8_t data_recv[2] = {0};
    spi_transaction_t sx1278_send = {
        .flags = 0,
        .length = 8 * sizeof(data_send),
        .rx_buffer = (void *)data_recv,
        .tx_buffer = (void *)data_send,
    };
    gpio_set_level(SX1278_NSS_PIN, 0);
    spi_device_transmit(spi_handle, &sx1278_send);
    gpio_set_level(SX1278_NSS_PIN, 1);
}

void sx1278_reset(void)
{
    gpio_set_level(SX1278_RST_PIN, 0);
    vTaskDelay(10 / portTICK_RATE_MS);
    gpio_set_level(SX1278_RST_PIN, 1);
    vTaskDelay(10 / portTICK_RATE_MS);
}

void sx1278_sleep(void)
{
    sx1278_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void sx1278_standby(void)
{
    sx1278_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void sx1278_rx_contiuous(void)
{
    sx1278_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

void sx1278_rx_single(void)
{
    sx1278_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
}

void sx1278_tx(void)
{
    sx1278_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
}

void sx1278_set_irq(uint8_t val)
{
    sx1278_write_reg(REG_DIO_MAPPING_1, val);
}

void sx1278_cad(void)
{
    sx1278_set_irq(0x80);
    sx1278_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_CAD);
}

void sx1278_set_tx_power(uint8_t output_power)
{
    if (output_power > 15)
    {
        ESP_LOGE(TAG, "Invalid output power");
        return;
    }
    // PA output pin: PA_BOOST pin
    sx1278_write_reg(REG_PA_CONFIG, PA_BOOST | output_power);
}

void sx1278_set_LNA_gain(uint8_t gain)
{
    if (gain > 6)
    {
        ESP_LOGW(TAG, "Invalid gain");
        return;
    }

    if (gain == 0)
        sx1278_write_reg(REG_MODEM_CONFIG_3, 0x04);
    else
    {
        sx1278_write_reg(REG_MODEM_CONFIG_3, 0x00);
        sx1278_write_reg(REG_LNA, sx1278_read_reg(REG_LNA) | (gain << 5));
    }
}

void sx1278_set_freq(uint64_t freq)
{
    uint64_t frf = ((uint64_t)freq << 19) / 32000000;
    sx1278_write_reg(REG_FRF_MSB, (uint8_t)(frf >> 16));
    sx1278_write_reg(REG_FRF_MID, (uint8_t)(frf >> 8));
    sx1278_write_reg(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

void sx1278_set_bandwidth(long band)
{
    int bw;
    if (band <= 7.8E3)
        bw = 0;
    else if (band <= 10.4E3)
        bw = 1;
    else if (band <= 15.6E3)
        bw = 2;
    else if (band <= 20.8E3)
        bw = 3;
    else if (band <= 31.25E3)
        bw = 4;
    else if (band <= 41.7E3)
        bw = 5;
    else if (band <= 62.5E3)
        bw = 6;
    else if (band <= 125E3)
        bw = 7;
    else if (band <= 250E3)
        bw = 8;
    else
        bw = 9;
    sx1278_write_reg(REG_MODEM_CONFIG_1, (sx1278_read_reg(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
}

void sx1278_set_sf(uint8_t sf)
{
    if (sf < 6 || sf > 12)
    {
        ESP_LOGE(TAG, "Invalid spreading factor");
        return;
    }

    if (sf == 6)
    {
        sx1278_write_reg(REG_DETECTION_OPTIMIZE, 0xc5);
        sx1278_write_reg(REG_DETECTION_THRESHOLD, 0x0c);
    }
    else
    {
        sx1278_write_reg(REG_DETECTION_OPTIMIZE, 0xc3);
        sx1278_write_reg(REG_DETECTION_THRESHOLD, 0x0a);
    }
    sx1278_write_reg(REG_MODEM_CONFIG_2, (sx1278_read_reg(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
}

void sx1278_set_cr(uint8_t cr)
{
    if (cr < 5 || cr > 8)
    {
        ESP_LOGE(TAG, "Invalid coding rate");
        return;
    }

    cr = cr - 4;
    sx1278_write_reg(REG_MODEM_CONFIG_1, (sx1278_read_reg(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

void sx1278_set_header(bool en, uint32_t size)
{
    if (en)
        sx1278_write_reg(REG_MODEM_CONFIG_1, sx1278_read_reg(REG_MODEM_CONFIG_1) & 0xfe);
    else
    {
        sx1278_write_reg(REG_MODEM_CONFIG_1, sx1278_read_reg(REG_MODEM_CONFIG_1) | 0x01);
        sx1278_write_reg(REG_PAYLOAD_LENGTH, size);
    }
}

void sx1278_set_crc(bool en)
{
    if (en)
        sx1278_write_reg(REG_MODEM_CONFIG_2, sx1278_read_reg(REG_MODEM_CONFIG_2) | 0x04);
    else
        sx1278_write_reg(REG_MODEM_CONFIG_2, sx1278_read_reg(REG_MODEM_CONFIG_2) & 0xfb);
}

void sx1278_set_preamble(int len)
{
    sx1278_write_reg(REG_PREAMBLE_MSB, (uint8_t)(len >> 8));
    sx1278_write_reg(REG_PREAMBLE_LSB, (uint8_t)(len >> 0));
}

int sx1278_get_rssi(void)
{
    return (sx1278_read_reg(REG_PKT_RSSI_VALUE) - 164);
}

float sx1278_get_snr(void)
{
    return ((int8_t)sx1278_read_reg(REG_PKT_SNR_VALUE) * 0.25);
}

void sx1278_init(void)
{
    sx1278_reset();
    uint8_t ver = sx1278_read_reg(REG_VERSION);
    ESP_LOGI(TAG, "SX1278 version: 0x%02x", ver);
    sx1278_sleep();
    sx1278_write_reg(REG_FIFO_RX_BASE_ADDR, 0x00);
    sx1278_write_reg(REG_FIFO_TX_BASE_ADDR, 0x00);
    sx1278_set_LNA_gain(0);
    sx1278_set_tx_power(13);
    sx1278_set_freq(433E6);
    sx1278_set_bandwidth(250E3);
    sx1278_set_sf(10U);
    sx1278_set_cr(5U);
    sx1278_set_preamble(12);
    sx1278_set_header(true, 0);
    sx1278_set_crc(true);
    // sx1278_set_irq(0x00);
    sx1278_standby();
}

void sx1278_send_data(uint8_t *data_send, int size)
{
    sx1278_standby();
    sx1278_write_reg(REG_FIFO_ADDR_PTR, 0);
    sx1278_write_reg(REG_PAYLOAD_LENGTH, 0);
    for (int index = 0; index < size; index++)
    {
        sx1278_write_reg(REG_FIFO, data_send[index]);
    }
    sx1278_write_reg(REG_PAYLOAD_LENGTH, size);
    // Start transmission and wait for conclusion
    sx1278_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
    while (!(sx1278_read_reg(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK))
    {
        vTaskDelay(10 / portTICK_RATE_MS);
    }
    int irq = sx1278_read_reg(REG_IRQ_FLAGS);
    sx1278_write_reg(REG_IRQ_FLAGS, irq);
    // sx1278_sleep();
}

void sx1278_start_recv_data(void)
{
    sx1278_set_irq(0x00);
    sx1278_rx_contiuous();
}

sx1278_err_t sx1278_recv_data(uint8_t *data_recv, uint32_t *len, int *rssi, float *snr, bool isStayinRX)
{
    memset((char *)data_recv, '\0', strlen((char *)data_recv));
    int irq = sx1278_read_reg(REG_IRQ_FLAGS);
    sx1278_write_reg(REG_IRQ_FLAGS, irq);

    if (!(irq & IRQ_RX_DONE_MASK))
    {
        ESP_LOGI(TAG, "Invalid RxDone Interrupt");
        return SX1278_INVALID_RX_DONE;
    }

    if (!(irq & IRQ_VALID_HEADER_MASK))
    {
        ESP_LOGI(TAG, "Invalid Header Interrupt");
        return SX1278_INVALID_HEADER;
    }

    if (irq & IRQ_PAYLOAD_CRC_ERROR_MASK)
    {
        ESP_LOGI(TAG, "Payload Crc Error Interrupt");
        return SX1278_PAYLOAD_CRC_ERROR;
    }

    *len = sx1278_read_reg(REG_RX_NB_BYTES);
    *rssi = sx1278_get_rssi();
    *snr = sx1278_get_snr();
    sx1278_write_reg(REG_FIFO_ADDR_PTR, sx1278_read_reg(REG_FIFO_RX_CURRENT_ADDR));
    for (int index = 0; index < *len; index++)
    {
        data_recv[index] = sx1278_read_reg(REG_FIFO);
    }
    if (isStayinRX == false)
        sx1278_standby();
    return SX1278_OK;
}
// sx1278_err_t sx1278_recv_data(uint8_t *data_recv, int *rssi, float *snr, sx1278_node_slot_t *node_slot)
// {
//     int irq = sx1278_read_reg(REG_IRQ_FLAGS);
//     memset((char *)data_recv, '\0', strlen((char *)data_recv));
//     sx1278_write_reg(REG_IRQ_FLAGS, irq);

//     if (!(irq & IRQ_RX_DONE_MASK))
//     {
//         ESP_LOGE(TAG, "Invalid RxDone Interrupt");
//         return SX1278_INVALID_RX_DONE;
//     }

//     if (!(irq & IRQ_VALID_HEADER_MASK))
//     {
//         ESP_LOGE(TAG, "Invalid Header Interrupt");
//         return SX1278_INVALID_HEADER;
//     }

//     if (irq & IRQ_PAYLOAD_CRC_ERROR_MASK)
//     {
//         ESP_LOGE(TAG, "Payload Crc Error Interrupt");
//         return SX1278_PAYLOAD_CRC_ERROR;
//     }

//     int len = sx1278_read_reg(REG_RX_NB_BYTES);
//     *rssi = sx1278_get_rssi();
//     *snr = sx1278_get_snr();
//     sx1278_standby();
//     sx1278_write_reg(REG_FIFO_ADDR_PTR, sx1278_read_reg(REG_FIFO_RX_CURRENT_ADDR));
//     for (int index = 0; index < len; index++)
//     {
//         data_recv[index] = sx1278_read_reg(REG_FIFO);
//     }
//     return parse_packet(data_recv, node_slot);
// }

int get_random_value(int min, int max)
{
    int random;
    if (max <= min)
    {
        ESP_LOGE(TAG, "Range error");
        return 0;
    }
    return random = min + rand() % (max + 1 - min);
}

uint8_t get_crc_value(uint8_t *data, int len)
{
    uint8_t crc = 0;
    for (int i = 0; i < len; i++)
    {
        crc ^= data[i];
    }
    return crc;
}

bool listen_before_talk(void)
{
    sx1278_standby();
    EventBits_t evt_bits;
    uint8_t irq;
    uint32_t timeout = xTaskGetTickCount();
    // is_LoRa_processing = false;
    while ((xTaskGetTickCount() - timeout) <= 4000)
    {
        xEventGroupClearBits(sx1278_evt_group, SX1278_DIO0_BIT);
        sx1278_cad();
        evt_bits = xEventGroupWaitBits(sx1278_evt_group, SX1278_DIO0_BIT, pdTRUE, pdFALSE, 1000);
        if ((evt_bits & SX1278_DIO0_BIT) != 0U)
        {
            // logPC("LoRa - CAD Done\tTook: %i ms\t", HAL_GetTick() - timeout);
            irq = sx1278_read_reg(REG_IRQ_FLAGS);
            sx1278_write_reg(REG_IRQ_FLAGS, irq);
            if ((irq & 0x01) != 0U)
            {
                // logPC("LoRa - CAD Detected\t");
                // HAL_Delay(get_random_value(0, 50));
            }
            else
            {
                // logPC("LoRa - CAD Clear\t");
                sx1278_set_irq(0x00);
                sx1278_standby();
                return true;
            }
        }
    }
    irq = sx1278_read_reg(REG_IRQ_FLAGS);
    sx1278_write_reg(REG_IRQ_FLAGS, irq);
    sx1278_set_irq(0x00);
    sx1278_standby();
    // logPC("Listen FAIL...\t");
    return false;
}
bool is_Node_ID_inNetwork(uint16_t Node_ID)
{
    // Node_Table[0].Link.Node_ID = 0xCAACU;
    // Node_Table[1].Link.Node_ID = 0xCAADU;
    // Node_Table[2].Link.Node_ID = 0xCAAEU;
    for (int i = 0; i < MAX_NODE; i++)
    {
        if (Node_Table[i].Link.Node_ID == Node_ID)
            return true;
    }
    return false;
}

uint8_t sizeof_node()
{
  for(uint8_t i=0 ; i< MAX_NODE; i++)
  {
    if( Node_Table[i].Link.Node_ID == 0)
    {
        return (i);
    }
  }
  return MAX_NODE;
}

void LoRa_Packet_Parser(const uint8_t *data, uint32_t len)
{
    switch ((uint16_t)((data[0] << 8) & 0xFF00) | (uint16_t)(data[1] & 0x00FF))
    { //   {
    case PACKET_ID_0:
        if (len != sizeof(Link_Packet_t))
            ESP_LOGI(TAG, "error");
        ESP_LOGI(TAG, "This is Link Pack");
        Link_Struct_t link_payload;
        memcpy((uint8_t *)&link_payload, (uint8_t *)&data[2], sizeof(Link_Struct_t));
        ESP_LOGI(TAG,"Node ID :%d",link_payload.Node_ID);
        if ((is_Node_ID_inNetwork(link_payload.Node_ID) == true) )
        {
            ResponsePacket_t resp = {PACKET_ID_2, link_payload.Node_ID, NORMAL_MODE, 180U, (uint16_t)(((LINK_ACCEPT << 8) & 0xFF00) | LINK_ACK)};
            if (listen_before_talk() == false)
            {
                sx1278_start_recv_data();
                break;
            }
            sx1278_send_data(&resp, sizeof(ResponsePacket_t));
            ESP_LOGI(TAG, "Done!\n");
        }
        else if( Check_Queue(xQueue,link_payload.Node_ID) == true )
        {
          if (flagmqtt == 1)
          {
            Node_Table[sizeof_node()].Link.Node_ID = link_payload.Node_ID;
            Pop_Queue(xQueue);
            ResponsePacket_t resp = {PACKET_ID_2, link_payload.Node_ID, NORMAL_MODE, 180U, (uint16_t)(((LINK_ACCEPT << 8) & 0xFF00) | LINK_ACK)};
            if (listen_before_talk() == false)
            {
                sx1278_start_recv_data();
                break;
            }
            sx1278_send_data(&resp, sizeof(ResponsePacket_t));
            ESP_LOGI(TAG, "Done2!\n");
            flagmqtt = 0 ;
          }
        }
        else
        {
          Push_Queue(xQueue,1,link_payload.Node_ID,0);
          char data[50];
          if (link_payload.Node_ID == 0xCAAC)
          {
            sprintf(data,"{\"node new \":0xCAAC}");
            esp_mqtt_client_publish(client, TOPIC , data, 0, 1, 0);
          }
          else if (link_payload.Node_ID == 0xCAAD)
          {
            sprintf(data,"{\"node new \":0xCAAD}");
            esp_mqtt_client_publish(client, TOPIC , data, 0, 1, 0);
          }
          else if (link_payload.Node_ID == 0xCAAE)
          {
            sprintf(data,"{\"node new \":0xCAAE}");
            esp_mqtt_client_publish(client, TOPIC , data, 0, 1, 0);
          }
            else if (link_payload.Node_ID == 0xCAAF)
          {
            sprintf(data,"{\"node new \":0xCAAF}");
            esp_mqtt_client_publish(client, TOPIC , data, 0, 1, 0);
          }

        }
        sx1278_start_recv_data();
        break;
    case PACKET_ID_1:
    {
        if (len != sizeof(Data_Packet_t))
            break;
        ESP_LOGI(TAG, "Data Packet\t");
        Data_Struct_t data_recv;
        memcpy((uint8_t *)&data_recv, (uint8_t *)&data[2], sizeof(Data_Struct_t));
        if (is_Node_ID_inNetwork(data_recv.Link.Node_ID) == true)
        {
             ESP_LOGI(TAG,"Node ID :%02X",data_recv.Link.Node_ID);
             ESP_LOGI(TAG,"Node ID :%d",data_recv.Node_Temp);
             char data[50];
             char data1[50];
            double a = (double)(((double)(data_recv.Node_Temp))/10);
            sprintf(data,"{\"node %02X\": %f}",data_recv.Link.Node_ID, a);
            esp_mqtt_client_publish(client, TOPIC , data, 0, 1, 0);
            sprintf(data1,"{\"batt %02X\": %d}",data_recv.Link.Node_ID,(data_recv.Link.Node_Battery_Voltage)/100);
            esp_mqtt_client_publish(client, TOPIC , data1, 0, 1, 0);
            ESP_LOGI(TAG, "TAKE_DATA_PACKET");
            ResponsePacket_t resp = {PACKET_ID_2, data_recv.Link.Node_ID, NORMAL_MODE, 180U, (uint16_t)(((LINK_CARRYON << 8) & 0xFF00) | LINK_ACK)};
            ESP_LOGI(TAG, "Response Ready...\t");
            if (listen_before_talk() == false)
            {
                sx1278_start_recv_data();
                break;
            }
            else
            {
                ESP_LOGI(TAG, "Sending packet...\t");
                sx1278_send_data(&resp, sizeof(ResponsePacket_t));

           }
        }
        else
        {
            ESP_LOGI(TAG, "NOT IN NETWORK");
            ResponsePacket_t resp = {PACKET_ID_2, data_recv.Link.Node_ID, LINK_MODE, 180U, (uint16_t)(((LINK_DISMISS << 8) & 0xFF00) | LINK_ACK)};
            if (listen_before_talk() == false)
                break;
            else
            {
                ESP_LOGI(TAG, "Sending packet...\t");
                sx1278_send_data(&resp, sizeof(ResponsePacket_t));
            }
            ESP_LOGI(TAG, "Done!\n");
            sx1278_start_recv_data();
        }
        sx1278_start_recv_data();
        break;
    }
    }
}

void sx1278_task(void *param)
{
    static uint8_t data[100] = {0};
    static uint32_t nByteRx = 0;
    static int rssi = -1;
    static float snr = -1;
    EventBits_t evt_bits;
    TickType_t period_tick;
    int slot_cnt = 0;
    sx1278_gpio_init();
    sx1278_spi_init();
    sx1278_init();

    sx1278_start_recv_data();

    sx1278_evt_group = xEventGroupCreate();
    // sx1278_network.total_slots = NW_DEFAULT_TOTAL_SLOTS;
    // sx1278_network.gate_id = 100;
    ESP_LOGI(TAG, "LoRa payload:\t");
    while (1)
    {
        
       
        // ESP_LOGI(TAG, "LoRa payload:\t");
        //  xEventGroupClearBits(sx1278_evt_group, SX1278_DIO0_BIT);sx1278_start_recv_data();sx1278_start_recv_data();
        // xEventGroupClearBits(sx1278_evt_group, SX1278_DIO0_BIT );
        evt_bits = xEventGroupWaitBits(sx1278_evt_group, SX1278_DIO0_BIT, pdTRUE, pdFALSE, 1000 / portTICK_PERIOD_MS);
        //ESP_LOGI(TAG, "LoRa payload:\t");
        static int check = 0;
        check++;
        //
        //ESP_LOGI(TAG,"RSSI:%d",rssi);
        if(check >=30)
        {
        if (sizeof_node() >= 0)
        {
            int i ;
        
            char data2[50];
            for(i= 0 ; i < sizeof_node(); i++ )
            {
            ESP_LOGI(TAG,"PUSH LIST");
            sprintf(data2,"{\"list%d\":%02X}",i,Node_Table[i].Link.Node_ID);
            esp_mqtt_client_publish(client, TOPIC , data2, 0, 1, 0);
            }
            for(int k = sizeof_node(); k < MAX_NODE;k ++)
            {
            sprintf(data2,"{\"list%d\":%02X}",k,0x00);
            esp_mqtt_client_publish(client, TOPIC , data2, 0, 1, 0);
            }
        }
        if( Sizeof_Queue_Current(xQueue) >= 0)
        {
            int j;
            char data1[50];
            for(j= 0 ; j < Sizeof_Queue_Current(xQueue); j++ )
            {
            ESP_LOGI(TAG,"PUSH LIST");
            sprintf(data1,"{\"queue%d\":%02X}",j,xQueue[j].data);
            esp_mqtt_client_publish(client, TOPIC , data1, 0, 1, 0);
            }

            for(int l = Sizeof_Queue_Current(xQueue); l < MAX_NODE;l ++){
            sprintf(data1,"{\"queue%d\":%02X}",l,0x00);
            esp_mqtt_client_publish(client, TOPIC , data1, 0, 1, 0);
            }
        }
        check =0;
        }
        if ((evt_bits & SX1278_DIO0_BIT))
        {
            if (sx1278_recv_data((uint8_t *)data, &nByteRx, &rssi, &snr, true) == SX1278_OK)
            {
                ESP_LOGE(TAG, "LoRa Received %d byte(s)\trssi: %d\tsnr: %2.1f\n", nByteRx, rssi, snr);
                //ESP_LOGI(TAG, "LoRa payload:\t");
                for (int i = 0; i < nByteRx; i++)
                {
                    //ESP_LOGI(TAG, "%02X ", data[i]);
                }
                LoRa_Packet_Parser((const uint8_t *)data, nByteRx);
            }
        }
        else
        {
            //ESP_LOGE(TAG, "Timeout");
        }
        // xEventGroupClearBits(sx1278_evt_group, SX1278_DIO0_BIT);
        // ResponsePacket_t resp = {PACKET_ID_2, 10, NORMAL_MODE, 180U, (uint16_t)(((LINK_CARRYON << 8) & 0xFF00) | LINK_ACK)};
        // sx1278_send_data(&resp, sizeof(ResponsePacket_t));
    }
}

void sx1279_task(void *param)
{
    
}
