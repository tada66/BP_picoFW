#include "UART.h"

char cmd_buffer[CMD_BUFFER_SIZE];
int cmd_buffer_index = 0;
uint8_t next_seq_num = 0;
uint8_t last_received_seq = 0;

pending_message_t pending_messages[MAX_PENDING_MSGS] = {0};

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

/*void send_uart_response(uint8_t cmd_type, const uint8_t *data, size_t data_length) {
    // Command format: START(0xAA) + CMD_TYPE + LEN + DATA + CRC8
    uint8_t seq_num = next_seq_num++;
    uint8_t header[4] = {0xAA, cmd_type, seq_num, data_length};
    uint8_t temp_buffer[256];
    
    memcpy(temp_buffer, header, 4);
    if (data_length > 0 && data != NULL) {
        memcpy(temp_buffer + 4, data, data_length);
    }
    
    // Calculate CRC8 over the entire message (header + data)
    uint8_t crc = calculate_crc8(temp_buffer, 4 + data_length);
    
    // Send the message over UART
    uart_write_blocking(UART_ID, header, 4);
    if (data_length > 0 && data != NULL) {
        uart_write_blocking(UART_ID, data, data_length);
    }
    uart_putc(UART_ID, crc);
    
    printf("Sent response: CMD=0x%02X, SEQ=%d, LEN=%d, CRC=0x%02X\n", cmd_type, seq_num, data_length, crc);
}*/

bool send_uart_command(uint8_t cmd_type, const uint8_t *data, size_t data_length) {
    // Find an available slot
    int slot = -1;
    for (int i = 0; i < MAX_PENDING_MSGS; i++) {
        if (!pending_messages[i].in_use) {
            slot = i;
            break;
        }
    }
    
    if (slot < 0) {
        printf("ERROR: No free slots for message tracking\n");
        return false;
    }
    
    // Prepare the message for tracking
    pending_message_t *msg = &pending_messages[slot];
    msg->in_use = true;
    msg->seq_num = next_seq_num++;
    msg->cmd_type = cmd_type;
    msg->data_length = data_length > 64 ? 64 : data_length;
    msg->sent_time = get_absolute_time();
    msg->retries = 0;
    
    if (data != NULL && data_length > 0) {
        memcpy(msg->data, data, msg->data_length);
    }
    
    // Send the initial transmission
    uint8_t header[4] = {0xAA, cmd_type, msg->seq_num, data_length};
    uint8_t temp_buffer[256];
    
    memcpy(temp_buffer, header, 4);
    if (data_length > 0 && data != NULL) {
        memcpy(temp_buffer + 4, data, data_length);
    }
    
    uint8_t crc = calculate_crc8(temp_buffer, 4 + data_length);
    
    uart_write_blocking(UART_ID, header, 4);
    if (data_length > 0 && data != NULL) {
        uart_write_blocking(UART_ID, data, data_length);
    }
    uart_putc(UART_ID, crc);
    
    printf("Sent command: CMD=0x%02X, SEQ=%d, LEN=%d, CRC=0x%02X\n", 
           cmd_type, msg->seq_num, data_length, crc);
    
    return true;
}

// Process message timeouts and retransmissions
void process_timeouts(void) {
    absolute_time_t now = get_absolute_time();
    
    for (int i = 0; i < MAX_PENDING_MSGS; i++) {
        pending_message_t *msg = &pending_messages[i];
        
        if (msg->in_use) {
            // Check if timed out
            if (absolute_time_diff_us(msg->sent_time, now) > (ACK_TIMEOUT_MS * 1000)) {
                // Timed out - retry or fail
                if (msg->retries < MAX_RETRANSMITS) {
                    // Retransmit the message
                    msg->retries++;
                    msg->sent_time = now;
                    
                    // Resend the message
                    uint8_t header[4] = {0xAA, msg->cmd_type, msg->seq_num, msg->data_length};
                    uint8_t temp_buffer[256];
                    
                    memcpy(temp_buffer, header, 4);
                    if (msg->data_length > 0) {
                        memcpy(temp_buffer + 4, msg->data, msg->data_length);
                    }
                    
                    uint8_t crc = calculate_crc8(temp_buffer, 4 + msg->data_length);
                    
                    uart_write_blocking(UART_ID, header, 4);
                    if (msg->data_length > 0) {
                        uart_write_blocking(UART_ID, msg->data, msg->data_length);
                    }
                    uart_putc(UART_ID, crc);
                    
                    printf("RETRANSMIT attempt %d: CMD=0x%02X, SEQ=%d\n", 
                           msg->retries, msg->cmd_type, msg->seq_num);
                } else {
                    // Max retries reached - give up
                    printf("ERROR: Message failed after %d retries: CMD=0x%02X, SEQ=%d\n", 
                           MAX_RETRANSMITS, msg->cmd_type, msg->seq_num);
                    msg->in_use = false;
                }
            }
        }
    }
}

void uart_background_task(void) {
    process_timeouts();
}

void send_ack(uint8_t seq_num) {
    // We don't need to track ACKs for retransmission since they're not ACKed themselves
    uint8_t header[4] = {0xAA, CMD_ACK, next_seq_num++, 1};
    uint8_t data[1] = {seq_num};
    uint8_t temp_buffer[256];
    
    memcpy(temp_buffer, header, 4);
    memcpy(temp_buffer + 4, data, 1);
    
    uint8_t crc = calculate_crc8(temp_buffer, 5);
    
    uart_write_blocking(UART_ID, header, 4);
    uart_write_blocking(UART_ID, data, 1);
    uart_putc(UART_ID, crc);
    
    printf("Sent ACK: SEQ=%d for message SEQ=%d\n", header[2], seq_num);
}

void on_uart_rx()
{
    // START 0xAA
    // CMD_TYPE
    // SEQUENCE NUMBER
    // DATA LENGTH
    // DATA (variable length)
    // CRC8 (calculated over CMD_TYPE, SEQUENCE NUMBER, DATA LENGTH, DATA)
    while (uart_is_readable(UART_ID)) {
        uint8_t c = uart_getc(UART_ID);
        if (cmd_buffer_index >= CMD_BUFFER_SIZE - 1) {
            printf("ERROR: Command buffer overflow!\n");
            cmd_buffer_index = 0;
            continue;
        }
        if (cmd_buffer_index == 0 && c != (char)0xAA) { // Guarantee we start with 0xAA
            continue;
        }
        cmd_buffer[cmd_buffer_index++] = c;
        if (cmd_buffer_index >= 4) {
            uint8_t cmd_type = cmd_buffer[1];
            uint8_t seq_num = cmd_buffer[2];
            uint8_t data_length = cmd_buffer[3];
            if (cmd_buffer_index == data_length + 5) { // DATA + START + CMD + SEQ + LEN + CHKSUM (crc8)
                // Validate CRC8 is correct
                uint8_t received_crc = (uint8_t)cmd_buffer[cmd_buffer_index - 1];
                uint8_t calculated_crc = calculate_crc8((uint8_t *)cmd_buffer, cmd_buffer_index - 1);
                if (received_crc != calculated_crc) {
                    printf("ERROR: CRC8 mismatch! Received: 0x%02X, Calculated: 0x%02X\n", 
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
                if(seq_num != (uint8_t)(last_received_seq + 1)) {
                    printf("Duplicate or out-of-order message SEQ=%d (last received SEQ=%d), ignoring\n", seq_num, last_received_seq);
                    cmd_buffer_index = 0; // Reset buffer index for next command
                    continue;
                }
                last_received_seq = seq_num;

                printf("Command received: CMD=0x%02X, SEQ=%d, Length=%d\n", cmd_type, seq_num, data_length);
                if(cmd_type != CMD_ACK) {
                    send_ack(seq_num);
                }
                switch (cmd_buffer[1]) {
                    case CMD_PING:
                        send_uart_command(CMD_PING, NULL, 0);
                        break;
                    case CMD_ACK:
                        if (data_length >= 1) {
                            uint8_t acked_seq = cmd_buffer[4];  // First data byte
                            for (int i = 0; i < MAX_PENDING_MSGS; i++) {
                                if (pending_messages[i].in_use && pending_messages[i].seq_num == acked_seq) {
                                    pending_messages[i].in_use = false;
                                    printf("Message SEQ=%d successfully acknowledged\n", acked_seq);
                                    break;
                                }
                            }
                        }
                        break;
                }
                cmd_buffer_index = 0; // Reset buffer index for next command
            }
        }

    }
}