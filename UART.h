#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>  // For printf debugging
#include "hardware/uart.h"
#include "PIN_ASSIGNMENTS.h"
#include "pico/time.h"

#define CRC8_POLYNOMIAL 0x07
#define CMD_BUFFER_SIZE 128
#define MAX_SEQ_DATA_LENGTH 256

#define ACK_TIMEOUT_MS 1000
#define MAX_RETRANSMITS 3
#define MAX_PENDING_MSGS 2

extern char cmd_buffer[CMD_BUFFER_SIZE];
extern int cmd_buffer_index;

extern uint8_t next_seq_num;
extern uint8_t last_received_seq;

enum Commands {
    CMD_PING = 0x01,
    CMD_ACK = 0x02,
    CMD_MOVE_ABS = 0x10,
    CMD_TRACK = 0x11,
    CMD_PAUSE = 0x12,
    CMD_RESUME = 0x13,
    CMD_GETPOS = 0x14,
    CMD_POSITION = 0x15,
    CMD_STATUS = 0x20,
    CMD_ESTOPTRIG = 0x21
};

// Message tracking structure
typedef struct {
    bool in_use;                 // Is this slot in use?
    uint8_t seq_num;             // Sequence number of this message
    uint8_t cmd_type;            // Command type
    uint8_t data[64];            // Message data
    size_t data_length;          // Length of data
    absolute_time_t sent_time;   // When the message was sent
    uint8_t retries;             // Number of retransmission attempts
} pending_message_t;

uint8_t calculate_crc8(const uint8_t *data, size_t length);
//void send_uart_response(uint8_t cmd_type, const uint8_t *data, size_t data_length);
void on_uart_rx();
void process_timeouts(void);
void uart_background_task(void);
bool send_uart_command(uint8_t cmd_type, const uint8_t *data, size_t data_length);