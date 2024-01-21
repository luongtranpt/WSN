/*
 * sx1278.h
 *
 *  Created on: Feb 8, 2023
 *      Author: dung
 */

#ifndef INC_SX1278_H_
#define INC_SX1278_H_
#include <stdbool.h>


// User definitions

#define HSPI            (&hspi1)
#define LoRa_RST_PORT   LoRa_RST_GPIO_Port
#define LoRa_RST_PIN    LoRa_RST_Pin
#define LoRa_CS_PORT    SPI1_CS_GPIO_Port
#define LoRa_CS_PIN     SPI1_CS_Pin



// Register definitions
#define REG_FIFO                        0x00
#define REG_OP_MODE                     0x01
#define REG_FRF_MSB                     0x06
#define REG_FRF_MID                     0x07
#define REG_FRF_LSB                     0x08
#define REG_PA_CONFIG                   0x09
#define REG_LNA                         0x0c
#define REG_FIFO_ADDR_PTR               0x0d
#define REG_FIFO_TX_BASE_ADDR           0x0e
#define REG_FIFO_RX_BASE_ADDR           0x0f
#define REG_FIFO_RX_CURRENT_ADDR        0x10
#define REG_IRQ_FLAGS                   0x12
#define REG_RX_NB_BYTES                 0x13
#define REG_PKT_SNR_VALUE               0x19
#define REG_PKT_RSSI_VALUE              0x1a
#define REG_MODEM_CONFIG_1              0x1d
#define REG_MODEM_CONFIG_2              0x1e
#define REG_PREAMBLE_MSB                0x20
#define REG_PREAMBLE_LSB                0x21
#define REG_PAYLOAD_LENGTH              0x22
#define REG_MODEM_CONFIG_3              0x26
#define REG_RSSI_WIDEBAND               0x2c
#define REG_DETECTION_OPTIMIZE          0x31
#define REG_DETECTION_THRESHOLD         0x37
#define REG_SYNC_WORD                   0x39
#define REG_DIO_MAPPING_1               0x40
#define REG_VERSION                     0x42

// Transceiver modes
#define MODE_LONG_RANGE_MODE            0x80
#define MODE_SLEEP                      0x00
#define MODE_STDBY                      0x01
#define MODE_TX                         0x03
#define MODE_RX_CONTINUOUS              0x05
#define MODE_RX_SINGLE                  0x06
#define MODE_CAD                        0x07

// PA configuration
#define PA_BOOST                        0x80

// IRQ masks
#define IRQ_TX_DONE_MASK                0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK      0x20
#define IRQ_RX_DONE_MASK                0x40
#define IRQ_VALID_HEADER_MASK           0x10
#define IRQ_CAD_DETECTED_MASK			0x01
#define IRQ_CAD_DONE_MASK				0x04

#define PA_OUTPUT_RFO_PIN               0
#define PA_OUTPUT_PA_BOOST_PIN          1

#define TIMEOUT_RESET                   100

#define DOWNLINK_RX_REQUEST_OPCODE 10
#define UPLINK_TX_RESPOND_OPCODE 20

#define SX1278_DIO0_BIT (1 << 0)
#define SX1278_DIO3_BIT (1 << 1)
#define SX1278_DIO4_BIT (1 << 2)

typedef int sx1278_opcode_type_t;

typedef enum
{
    SX1278_OK,
    SX1278_NOT_OK,
    SX1278_INVALID_RX_DONE,
    SX1278_PAYLOAD_CRC_ERROR,
    SX1278_INVALID_HEADER,
} sx1278_err_t;

typedef struct
{
    char opcode[5];
    char node_id[5];
    char gate_id[5];
    char period[5];
    char threshold[10];
    uint8_t crc;
} sx1278_packet_t;

typedef struct
{
    int node_id;
    int gate_id;
    char temp[10];
    char battery[10];
    int period;
    char threshold[10];
} sx1278_node_t;

uint8_t sx1278_read_reg(uint8_t reg);
void sx1278_write_reg(uint8_t reg, uint8_t val);
void sx1278_reset(void);
void sx1278_sleep(void);
void sx1278_standby(void);
void sx1278_rx_contiuous(void);
void sx1278_rx_single(void);
void sx1278_tx(void);
void sx1278_cad(void);
void sx1278_set_tx_power(uint8_t output_power);
void sx1278_set_LNA_gain(uint8_t gain);
void sx1278_set_freq(uint64_t freq);
void sx1278_set_bandwidth(long band);
void sx1278_set_sf(uint8_t sf);
void sx1278_set_cr(uint8_t cr);
void sx1278_set_header(bool en, uint32_t size);
void sx1278_set_crc(bool en);
void sx1278_set_preamble(int len);
int sx1278_get_rssi(void);
float sx1278_get_snr(void);
void sx1278_set_irq(uint8_t val);
void sx1278_init(void);
void sx1278_send_data(uint8_t *data_send, int size);
void sx1278_start_recv_data(void);
sx1278_err_t sx1278_recv_data(uint8_t *data_recv, uint32_t *len, int *rssi, float *snr, bool isStayinRX);
sx1278_err_t parse_packet(uint8_t *packet_data, sx1278_node_t *node);
int get_random_value(uint32_t seed, int min, int max);
bool listen_before_talk(void);
uint8_t get_crc_value(uint8_t *data, int len);
void send_respond(sx1278_opcode_type_t opcode, sx1278_node_t node, uint8_t *packet);

#endif /* INC_SX1278_H_ */
