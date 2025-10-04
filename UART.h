#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>  // For printf debugging
#include "hardware/uart.h"
#include "PIN_ASSIGNMENTS.h"

#define CRC8_POLYNOMIAL 0x07
#define CMD_BUFFER_SIZE 128
char cmd_buffer[CMD_BUFFER_SIZE];
int cmd_buffer_index = 0;

enum Commands {
    CMD_PING = 0x01,
    CMD_READY = 0x02,
    CMD_MOVE_ABS = 0x10,
    CMD_TRACK = 0x11,
    CMD_PAUSE = 0x12,
    CMD_RESUME = 0x13,
    CMD_GETPOS = 0x14,
    CMD_POSITION = 0x15,
    CMD_STATUS = 0x20,
    CMD_ESTOPTRIG = 0x21
};

uint8_t calculate_crc8(const uint8_t *data, size_t length);
void send_uart_response(uint8_t cmd_type, const uint8_t *data, size_t data_length);
void on_uart_rx();