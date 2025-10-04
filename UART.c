#include "UART.h"

uint8_t calculate_crc8(const uint8_t *data, size_t length) {
    uint8_t crc = 0xFF; // Initial value
    
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            } else {
                crc <<= 1;
            }
            crc &= 0xFF;  // Keep only the lower 8 bits
        }
    }
    
    return crc;
}

void send_uart_response(uint8_t cmd_type, const uint8_t *data, size_t data_length) {
    // Command format: START(0xAA) + CMD_TYPE + LEN + DATA + CRC8
    uint8_t header[3] = {0xAA, cmd_type, data_length};
    uint8_t temp_buffer[256];
    
    memcpy(temp_buffer, header, 3);
    if (data_length > 0 && data != NULL) {
        memcpy(temp_buffer + 3, data, data_length);
    }
    
    // Calculate CRC8 over the entire message (header + data)
    uint8_t crc = calculate_crc8(temp_buffer, 3 + data_length);
    
    // Send the message over UART
    uart_write_blocking(UART_ID, header, 3);
    if (data_length > 0 && data != NULL) {
        uart_write_blocking(UART_ID, data, data_length);
    }
    uart_putc(UART_ID, crc);
    
    printf("Sent response: CMD=0x%02X, LEN=%d, CRC=0x%02X\n", cmd_type, data_length, crc);
}

void on_uart_rx()
{
    // First byte is START byte 0xAA
    // Second byte is command type (defined in enum Commands)
    // Third byte is data length
    // DATA bytes follow
    // Last byte is checksum
    while (uart_is_readable(UART_ID)) {
        uint8_t c = uart_getc(UART_ID);
        if (cmd_buffer_index >= CMD_BUFFER_SIZE - 1) {
            fprintf(stderr, "ERROR: Command buffer overflow!\n");
            cmd_buffer_index = 0;
            continue;
        }
        if (cmd_buffer_index == 0 && c != (char)0xAA) { // Guarantee we start with 0xAA
            continue;
        }
        cmd_buffer[cmd_buffer_index++] = c;
        if (cmd_buffer_index >= 3) {
            int data_length = (unsigned char)cmd_buffer[2];
            if (cmd_buffer_index == data_length + 4) { // DATA + START + CMD + LEN + CHKSUM (crc8)
                // Validate CRC8 is correct
                uint8_t received_crc = (uint8_t)cmd_buffer[cmd_buffer_index - 1];
                uint8_t calculated_crc = calculate_crc8((uint8_t *)cmd_buffer, cmd_buffer_index - 1);
                if (received_crc != calculated_crc) {
                    fprintf(stderr, "ERROR: CRC8 mismatch! Received: 0x%02X, Calculated: 0x%02X\n", 
                           received_crc, calculated_crc);

                    // Try to resync by looking for next 0xAA
                    int i = 1;
                    while (i < cmd_buffer_index && cmd_buffer[i] != 0xAA) 
                        i++;
                    if (i < cmd_buffer_index) { // Cleanup
                        memmove(cmd_buffer, &cmd_buffer[i], cmd_buffer_index - i);
                        cmd_buffer_index -= i;
                    } else {
                        cmd_buffer_index = 0;
                    }
                    continue;
                }

                printf("Command received: 0x%02X, Length: %d\n", cmd_buffer[1], data_length);
                // Process fast commands here, movements will be handled in main loop to avoid blocking
                switch (cmd_buffer[1]) {
                    case CMD_PING:
                        send_uart_response(CMD_PING, NULL, 0);
                        break;
                }
                cmd_buffer_index = 0; // Reset buffer index for next command
            }
        }

    }
}