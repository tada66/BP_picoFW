#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/dma.h"
#include "PIN_ASSIGNMENTS.h"
#include "pico/time.h"
#include "pico/stdlib.h"
#include "STEPPER.h"
#include "DEBUGPRINT.h"

#define CRC8_POLYNOMIAL 0x07
#define CMD_BUFFER_SIZE 128
#define TX_BUFFER_SIZE 256  // Buffer for DMA transmission

#define MAX_RESPONSES 4
#define ACK_TIMEOUT_MS 1000
#define MAX_RETRANSMITS 3
#define MAX_MISSED_ACKS 2
#define BAUD_RATE 9600

extern int uart_tx_dma_channel;

enum Commands {
    CMD_ACK = 0x01,
    CMD_MOVE_STATIC = 0x10,
    CMD_MOVE_TRACKING = 0x11,
    CMD_PAUSE = 0x12,
    CMD_RESUME = 0x13,
    CMD_STOP = 0x14,
    CMD_TRACK_CELESTIAL = 0x15,  // Autonomous celestial tracking with alignment matrix
    CMD_GETPOS = 0x20,
    CMD_POSITION = 0x21,
    CMD_STATUS = 0x22,
    CMD_ESTOPTRIG = 0x30
};

// Message tracking structure
typedef struct {
    bool in_use;                 // Are we currently waiting for an ACK for this message?
    uint8_t msg_id;              // Somewhat random message ID - must be different from last message as its used to detect duplicates
    uint8_t cmd_type;            // Command type
    uint8_t data[64];            // Message data
    size_t data_length;          // Length of data
    absolute_time_t sent_time;   // When the message was sent
    uint8_t retries;             // Number of retransmission attempts
} pending_message_t;

#define RESPONSE_DATA_SIZE 32
typedef struct {
    uint8_t command;
    uint8_t data[RESPONSE_DATA_SIZE];
    uint8_t data_length;
    bool ready;
} response_message_t;

void uart_init_protocol();
uint8_t calculate_crc8(const uint8_t *data, size_t length);
void on_uart_rx();
void process_timeouts();
void uart_background_task();
bool send_uart_command(uint8_t cmd_type, const uint8_t *data, size_t data_length);
void send_uart_message(pending_message_t *msg);
void send_ack(uint8_t seq_num);
void on_uart_tx_dma_complete();
uint8_t generate_msg_id();
size_t cobsEncode(const void *data, size_t length, uint8_t *buffer);
size_t cobsDecode(const uint8_t *buffer, size_t length, void *data);
void process_responses(void);
void queue_response(uint8_t cmd_type, const uint8_t *data, size_t data_length);

#endif // UART_H