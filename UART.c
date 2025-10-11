#include "UART.h"

//TODO: Add COBS (https://www.embeddedrelated.com/showarticle/113.php) (or COBS-R https://pythonhosted.org/cobs/cobsr-intro.html) encoding to avoid 0xAA in data
//      the receiver currently gets confused if 0xAA appears anywhere other than the start of a message
//      this could also be solved by making the receiver more resistant to wild 0xAA bytes in the stream
//      but COBS (or potentially SLIP) is a more professional solution

//TODO: Consider replacing sequence numbers with some message ID system that would just check if the new message isnt the same as the last one
//      this would allow for duplicate message detection and also for ACKing specific messages.
//      we never send more messages than one at a time without receiving an ACK so sequence numbers are unnecessary complexity
//      as messages out of order are impossible in this system. IDs could just be randomly generated,
//      or a hash of the message contents, we only care about them not being the same as the last one


int missed_acks = 0;
uint8_t next_seq_num = 0;
uint8_t last_received_seq = 0xFF; // Last received sequence number, initialized to 0xFF so first valid is 0

pending_message_t pending_message;
uint8_t tx_buffer[TX_BUFFER_SIZE]; // Buffer for DMA transmission
volatile bool tx_busy = false;     // Flag to indicate if DMA TX is in progress

int uart_tx_dma_channel = -1;      // DMA channel for UART TX

void uart_init_protocol(void) {
    // Initialize UART
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    irq_set_exclusive_handler(UART0_IRQ, on_uart_rx);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(UART_ID, true, false);

    uart_tx_dma_channel = dma_claim_unused_channel(true);
    
    if (uart_tx_dma_channel < 0) {
        printf("ERROR: Could not claim DMA channel for UART TX!\n");
        return;
    }

    // Configure the DMA channel for UART TX
    dma_channel_config dma_conf = dma_channel_get_default_config(uart_tx_dma_channel);
    channel_config_set_transfer_data_size(&dma_conf, DMA_SIZE_8);           // 8-bit transfers
    channel_config_set_read_increment(&dma_conf, true);                      // Increment read address
    channel_config_set_write_increment(&dma_conf, false);                    // Don't increment write address
    channel_config_set_dreq(&dma_conf, uart_get_dreq(UART_ID, true));       // UART TX DREQ as the DMA trigger
    
    // Set up the DMA channel
    dma_channel_configure(
        uart_tx_dma_channel,
        &dma_conf,
        &uart_get_hw(UART_ID)->dr,   // Write to UART data register
        NULL,                        // Initial read address will be set later
        0,                           // Initial transfer count will be set later
        false                        // Don't start yet
    );
    
    // Set up DMA completion interrupt
    dma_channel_set_irq0_enabled(uart_tx_dma_channel, true);
    irq_set_exclusive_handler(DMA_IRQ_0, on_uart_tx_dma_complete);
    irq_set_enabled(DMA_IRQ_0, true);
    
    printf("UART protocol initialized with DMA for TX (channel %d)\n", uart_tx_dma_channel);
}

// DMA complete irq handler
void on_uart_tx_dma_complete(void) {
    dma_hw->ints0 = 1u << uart_tx_dma_channel;
    tx_busy = false;
}

uint8_t calculate_crc8(const uint8_t *data, size_t length) {
    uint8_t crc = 0xFF;

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
    // Wait until any previous DMA transfer is complete
    while (tx_busy) {
        tight_loop_contents();
    }
    
    // Prepare the message in the TX buffer
    uint8_t header[4] = {0xAA, msg->cmd_type, msg->seq_num, msg->data_length};
    size_t tx_size = 0;
    
    memcpy(tx_buffer, header, 4);
    tx_size += 4;
    
    if (msg->data_length > 0 && msg->data != NULL) {
        memcpy(tx_buffer + tx_size, msg->data, msg->data_length);
        tx_size += msg->data_length;
    }
    
    uint8_t crc = calculate_crc8(tx_buffer, tx_size);
    tx_buffer[tx_size++] = crc;
    
    // Set up and start DMA transfer
    dma_channel_set_read_addr(uart_tx_dma_channel, tx_buffer, false);
    dma_channel_set_trans_count(uart_tx_dma_channel, tx_size, false);
    
    // Mark TX as busy before starting DMA
    tx_busy = true;
    
    // Start the DMA transfer
    dma_channel_start(uart_tx_dma_channel);
    
    printf("Sent command via DMA: CMD=0x%02X, SEQ=%d, LEN=%d, CRC=0x%02X\n", 
        msg->cmd_type, msg->seq_num, msg->data_length, crc);
}

// Process message timeouts and retransmissions
void process_timeouts(void) {
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
                    printf("CRITICAL ERROR: 5 consecutive messages lost, resetting communication state\n");
                    // Reset all pending messages
                    pending_message.in_use = false;
                    last_received_seq = 0xFF;  // Reset to initial state
                    missed_acks = 0;
                }
            }
        }
    }
}

void uart_background_task() {
    process_timeouts();
}

void send_ack(uint8_t seq_num) {
    pending_message_t ack_msg;
    ack_msg.in_use = false; // No tracking needed for ACKs
    ack_msg.seq_num = seq_num;
    ack_msg.cmd_type = CMD_ACK;
    ack_msg.data_length = 1;
    ack_msg.data[0] = seq_num; // Echo back the sequence number being acknowledged
    send_uart_message(&ack_msg);
}

// UART RX interrupt handler (unchanged, but with command handling expanded)
void on_uart_rx(void) {
    static char cmd_buffer[CMD_BUFFER_SIZE];
    static int cmd_buffer_index = 0;

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
            if (cmd_buffer_index == data_length + 5) { // Whole command received (5 = DATA + START + CMD + SEQ + LEN + CHKSUM (crc8))
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

                if (seq_num != (uint8_t)(last_received_seq + 1)) {
                    printf("Out-of-order message SEQ=%d (expected %d), ignoring\n", 
                            seq_num, (uint8_t)(last_received_seq + 1));
                    cmd_buffer_index = 0;
                    continue;
                }
                last_received_seq = seq_num;
                printf("Command received: CMD=0x%02X, SEQ=%d, Length=%d\n", cmd_type, seq_num, data_length);
                if (cmd_type != CMD_ACK)
                    send_ack(seq_num);
                
                // Process commands
                switch (cmd_type) {
                    case CMD_ACK:
                        if (data_length >= 1) {
                            uint8_t acked_seq = cmd_buffer[4];  // First data byte
                            if (pending_message.in_use && pending_message.seq_num == acked_seq) {
                                pending_message.in_use = false;
                                printf("Message SEQ=%d successfully acknowledged\n", acked_seq);
                                missed_acks = 0;
                            }
                        }
                        break;

                    case CMD_PAUSE:
                        printf("Received PAUSE command\n");
                        break;
                        
                    case CMD_RESUME:
                        printf("Received RESUME command\n");
                        break;
                }
                
                cmd_buffer_index = 0;
            }
        }
    }
}