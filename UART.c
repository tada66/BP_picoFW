#include "UART.h"

uint8_t next_seq_num = 0;
uint8_t last_received_seq = 0;
int state = STATE_READY_TO_CONNECT;

pending_message_t pending_message;

void uart_init_protocol(){
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    irq_set_exclusive_handler(UART0_IRQ, on_uart_rx);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(UART_ID, true, false);
}

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

// Send a command with tracking for ACK and retransmission
bool send_uart_command(uint8_t cmd_type, const uint8_t *data, size_t data_length) {
    if (state != STATE_CONNECTED && cmd_type != CMD_PING) {
        printf("ERROR: Cannot send command 0x%02X, not connected! Only pings allowed to establish a connection!\n", cmd_type);
        return false;
    }
    if (pending_message.in_use) {
        printf("ERROR: Cannot send command 0x%02X, previous message still pending ACK!\n", cmd_type);
        return false;
    }

    // Prepare the message for tracking
    pending_message.in_use = true;
    pending_message.seq_num = next_seq_num++;
    pending_message.cmd_type = cmd_type;
    pending_message.data_length = data_length > 64 ? 64 : data_length;
    pending_message.sent_time = get_absolute_time();
    pending_message.retries = 0;
    
    if (data != NULL && data_length > 0) {
        memcpy(pending_message.data, data, pending_message.data_length);
    }
    send_uart_message(&pending_message);
    
    return true;
}

void send_uart_message(pending_message_t *msg) {
    // Send the initial transmission
    uint8_t header[4] = {0xAA, msg->cmd_type, msg->seq_num, msg->data_length};
    uint8_t temp_buffer[256];
    
    memcpy(temp_buffer, header, 4);
    if (msg->data_length > 0 && msg->data != NULL) {
        memcpy(temp_buffer + 4, msg->data, msg->data_length);
    }
    
    uint8_t crc = calculate_crc8(temp_buffer, 4 + msg->data_length);
    
    uart_write_blocking(UART_ID, header, 4);
    if (msg->data_length > 0 && msg->data != NULL) {
        uart_write_blocking(UART_ID, msg->data, msg->data_length);
    }
    uart_putc(UART_ID, crc);
    
    printf("Sent command: CMD=0x%02X, SEQ=%d, LEN=%d, CRC=0x%02X\n", 
        msg->cmd_type, msg->seq_num, msg->data_length, crc);
}

// Process message timeouts and retransmissions
void process_timeouts() {
    static int missed_acks = 0;
    absolute_time_t now = get_absolute_time();
     
    if (pending_message.in_use) {
        if (absolute_time_diff_us(pending_message.sent_time, now) > (ACK_TIMEOUT_MS * 1000)) {
            // Timed out - retry or fail
            if (pending_message.retries < MAX_RETRANSMITS) {
                // Retransmit the message
                pending_message.retries++;
                pending_message.sent_time = now;
                
                send_uart_message(&pending_message);
                
                printf("RETRANSMIT attempt %d: CMD=0x%02X, SEQ=%d\n", 
                        pending_message.retries, pending_message.cmd_type, pending_message.seq_num);
            } else {
                // Max retries reached - give up
                printf("ERROR: Message failed after %d retries: CMD=0x%02X, SEQ=%d\n", 
                        MAX_RETRANSMITS, pending_message.cmd_type, pending_message.seq_num);
                pending_message.in_use = false;

                missed_acks++;
                if (missed_acks >= MAX_MISSED_ACKS) {
                    // TODO: implement a state machine with a startup state, to which the system will return upon no connection
                    printf("CRITICAL ERROR: 5 consecutive messages lost, resetting communication state\n");
                    // Reset all pending messages
                    pending_message.in_use = false;
                    last_received_seq = 0;
                    state = STATE_READY_TO_CONNECT;
                }
            }
        }
    }
}

void uart_background_task() {
    static absolute_time_t last_ping_time = {0};

    switch (state){
        case STATE_READY_TO_CONNECT:
            absolute_time_t now = get_absolute_time();
            if (absolute_time_diff_us(last_ping_time, now) > 5000000) { // Every 5 seconds
                send_uart_command(CMD_PING, NULL, 0);
                last_ping_time = now;
            }
            break;
        case STATE_CONNECTED:
            process_timeouts();
            break;
    }
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
    static char cmd_buffer[CMD_BUFFER_SIZE];
    static int cmd_buffer_index = 0;
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
                        if(state == STATE_READY_TO_CONNECT) {
                            state = STATE_CONNECTED;
                            printf("Connection established!\n");
                        }
                        if (data_length >= 1) {
                            uint8_t acked_seq = cmd_buffer[4];  // First data byte
                            if (pending_message.in_use && pending_message.seq_num == acked_seq) {
                                pending_message.in_use = false;
                                printf("Message SEQ=%d successfully acknowledged\n", acked_seq);
                                break;
                            }
                        }
                        break;
                }
                cmd_buffer_index = 0; // Reset buffer index for next command
            }
        }

    }
}