/**
 * @file sx1278.h
 * @author Vanperdung (dung.nv382001@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-01-29
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#ifndef _SX1278_H_
#define _SX1278_H_

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#define SX1278_MOSI_PIN GPIO_NUM_23
#define SX1278_MISO_PIN GPIO_NUM_19
#define SX1278_NSS_PIN GPIO_NUM_14
#define SX1278_SCK_PIN GPIO_NUM_18
#define SX1278_DIO0_PIN GPIO_NUM_32
#define SX1278_DIO4_PIN GPIO_NUM_35
#define SX1278_DIO3_PIN GPIO_NUM_34
#define SX1278_RST_PIN GPIO_NUM_33

#define SX1278_DIO0_BIT BIT0
#define SX1278_DIO3_BIT BIT1
#define SX1278_DIO4_BIT BIT2

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

#define ACK 0x06
#define NACK 0x15

#define UPLINK_TX_REQUEST_OPCODE 10
#define DOWNLINK_RX_DATA_OPCODE 20

#define BUFF "VANPERDUNG VANPERDUNG VANPERDUNG VANPERDUNG"

#define NW_DEFAULT_TOTAL_SLOTS 10
#define NW_DEFAULT_PERIOD 5
#define NW_DEFAULT_THRESHOLD 25.0

#define PACKET_ID_0   (0xAAAAU)
#define PACKET_ID_1   (0x5A5AU)
#define PACKET_ID_2   (0x5555U)

#define LINK_ACCEPT   (0xAAU)
#define LINK_REJECT   (0x55U)

#define LINK_CARRYON  (0x00U)
#define LINK_DISMISS  (0xFFU)

#define LINK_ACK      (0xABU)
#define LINK_NACK     (0xBAU)

#define DEFAULT_PERIOD_SEC  (300U)

// DO NOT CHANGE
// #define EVT_LORA      (0x04U)
// #define EVT_TOCUH     (0x02U)

// DO NOT CHANGE
#define MAX_NODE      (6U)



typedef struct LoRa_Link_Struct
{
  uint16_t Node_ID;
  uint16_t Node_Status; 
  uint16_t Node_Battery_Voltage;
  uint16_t Node_Period;
} Link_Struct_t;

typedef struct LoRa_Data_Struct
{
  Link_Struct_t Link;
  uint16_t Node_Temp;
} Data_Struct_t;

typedef struct LoRa_Packet_ID_0
{
  uint16_t Packet_ID;
  Link_Struct_t Payload;
} Link_Packet_t;

typedef struct LoRa_Packet_ID_1
{
  uint16_t Packet_ID;
  Data_Struct_t Payload;
} Data_Packet_t;

typedef struct LoRa_Packet_ID_2
{
  uint16_t Packet_ID;
  uint16_t Target_Node_ID;
  uint16_t Target_Node_Status; 
  uint16_t Target_Node_Period;
  uint16_t Target_Node_Response;
} ResponsePacket_t;

typedef enum NodeStatus
{
  WAKEUP_MODE = 1U,
  LINK_MODE = 2U,
  NORMAL_MODE = 4U,
  RETRY_MODE = 8U,
  SHUTDOWN_MODE = 16U
} NodeStatus_t;

typedef int sx1278_opcode_type_t;

typedef enum
{
    SX1278_OK,
    SX1278_NOT_OK,
    SX1278_INVALID_RX_DONE,
    SX1278_PAYLOAD_CRC_ERROR,
    SX1278_INVALID_HEADER
} sx1278_err_t;

typedef struct 
{
    int node_id;
    int slot_id;
    int period; 
    char threshold[10];
    char temp[10];
    char battery[10];
} sx1278_node_slot_t;

typedef struct 
{
    char opcode[5];
    char node_id[5];
    char gate_id[5];
    char temp[10];
    char battery[10];
    char period[5];
    char threshold[10];
    uint8_t crc;
} sx1278_packet_t;

typedef struct 
{
    float threshold[10];
    int period;
} sx1278_attr_cfg_t;

typedef struct 
{
    bool network_run;
} sx1278_flag_t;

typedef struct 
{
    int total_slots;
    int gate_id;
    sx1278_node_slot_t node_slots[NW_DEFAULT_TOTAL_SLOTS];
    sx1278_flag_t flags;
} sx1278_network_t;

static Data_Struct_t Node_Table[MAX_NODE] = {0};

void sx1278_task(void *param);

//sx1278_err_t parse_packet(uint8_t *packet_data, sx1278_node_slot_t *node_slot);
#endif