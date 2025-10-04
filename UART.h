#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>  // For printf debugging
#include "hardware/uart.h"
#include "PIN_ASSIGNMENTS.h"

#define CRC8_POLYNOMIAL 0x07


uint8_t calculate_crc8(const uint8_t *data, size_t length);
void send_uart_response(uint8_t cmd_type, const uint8_t *data, size_t data_length);