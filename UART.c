#include "UART.h"


int missed_acks = 0;
uint8_t last_received_id = 0x00; // Last received message ID, a new message must have a different ID from the last one, 0x00 is invalid

pending_message_t pending_message;
uint8_t tx_buffer[TX_BUFFER_SIZE]; // Buffer for DMA transmission
volatile bool tx_busy = false;     // Flag to indicate if DMA TX is in progress

int uart_tx_dma_channel = -1;      // DMA channel for UART TX

response_message_t response_queue[MAX_RESPONSES];

void uart_init_protocol(void) {

    for (int i = 0; i < MAX_RESPONSES; i++) {
        response_queue[i].ready = false;
    }

    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    irq_set_exclusive_handler(UART0_IRQ, on_uart_rx);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(UART_ID, true, false);

    uart_tx_dma_channel = dma_claim_unused_channel(true);
    
    if (uart_tx_dma_channel < 0) {
        DEBUG_PRINT("ERROR: Could not claim DMA channel for UART TX!\n");
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
    
    DEBUG_PRINT("UART protocol initialized with DMA for TX (channel %d)\n", uart_tx_dma_channel);
}

// DMA complete irq handler
void on_uart_tx_dma_complete(void) {
    dma_hw->ints0 = 1u << uart_tx_dma_channel;
    tx_busy = false;
}

uint8_t generate_msg_id() {
    static uint8_t last_sent_id = 0x00; // Last sent message ID, initialized to invalid value
    uint8_t new_id;
    do {
        new_id = (uint8_t)(rand() % 256);
    } while (new_id == last_sent_id || new_id == 0x00); // Ensure it's different from last and not 0x00
    last_sent_id = new_id;
    return new_id;
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


/*
 * COBS (Consistent Overhead Byte Stuffing) Implementation
 * 
 * Source: Wikipedia - "Consistent Overhead Byte Stuffing"
 * URL: https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing
 * 
 * Functions cobsEncode() and cobsDecode() are based on the reference
 * implementation provided in the Wikipedia article.
 * 
 * Last accessed: 2nd November 2025
 */
size_t cobsEncode(const void *data, size_t length, uint8_t *buffer)
{
	uint8_t *encode = buffer; // Encoded byte pointer
	uint8_t *codep = encode++; // Output code pointer
	uint8_t code = 1; // Code value

	for (const uint8_t *byte = (const uint8_t *)data; length--; ++byte)
	{
		if (*byte) // Byte not zero, write it
			*encode++ = *byte, ++code;

		if (!*byte || code == 0xff) // Input is zero or block completed, restart
		{
			*codep = code, code = 1, codep = encode;
			if (!*byte || length)
				++encode;
		}
	}
	*codep = code; // Write final code value

	return (size_t)(encode - buffer);
}

size_t cobsDecode(const uint8_t *buffer, size_t length, void *data)
{
	const uint8_t *byte = buffer; // Encoded input byte pointer
	uint8_t *decode = (uint8_t *)data; // Decoded output byte pointer

	for (uint8_t code = 0xff, block = 0; byte < buffer + length; --block)
	{
		if (block) // Decode block byte
			*decode++ = *byte++;
		else
		{
			block = *byte++;             // Fetch the next block length
			if (block && (code != 0xff)) // Encoded zero, write it unless it's delimiter.
				*decode++ = 0;
			code = block;
			if (!code) // Delimiter code found
				break;
		}
	}

	return (size_t)(decode - (uint8_t *)data);
}

// Send a command with tracking for ACK and retransmission
bool send_command(uint8_t cmd_type, const uint8_t *data, size_t data_length) {
    if (cmd_type == CMD_ACK) {
        send_ack(data[0]); // Directly send ACK without tracking
        return true;
    }
    if (pending_message.in_use) {
        DEBUG_PRINT("ERROR: Cannot send command 0x%02X, previous message still pending ACK!\n", cmd_type);
        return false;
    }

    // Prepare the message for tracking
    pending_message.in_use = true;
    pending_message.msg_id = generate_msg_id();
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
    
    memset(tx_buffer, 0, sizeof(tx_buffer));
    
    // Prepare the message in the TX buffer
    uint8_t raw_buffer[TX_BUFFER_SIZE - 10];
    size_t raw_buf_size = 0;

    raw_buffer[raw_buf_size++] = msg->cmd_type;
    raw_buffer[raw_buf_size++] = msg->msg_id;
    raw_buffer[raw_buf_size++] = msg->data_length;

    if (msg->data_length > 0 && msg->data != NULL) {
        memcpy(raw_buffer + raw_buf_size, msg->data, msg->data_length);
        raw_buf_size += msg->data_length;
    }

    uint8_t crc = calculate_crc8(raw_buffer, raw_buf_size);
    raw_buffer[raw_buf_size++] = crc;

    size_t encoded_size = cobsEncode(raw_buffer, raw_buf_size, tx_buffer);
    tx_buffer[encoded_size] = 0x00; // COBS delimiter
    
    // Set up and start DMA transfer
    dma_channel_set_read_addr(uart_tx_dma_channel, tx_buffer, true);
    dma_channel_set_trans_count(uart_tx_dma_channel, encoded_size + 1, true);
    
    // Mark TX as busy before starting DMA
    tx_busy = true;
    
    // Start the DMA transfer
    dma_channel_start(uart_tx_dma_channel);

    DEBUG_PRINT("Sent: CMD=0x%02X, ID=0x%02X, LEN=%d, CRC=0x%02X\n", 
        msg->cmd_type, msg->msg_id, msg->data_length, crc);
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

                DEBUG_PRINT("RETRANSMIT attempt %d: CMD=0x%02X, ID=0x%02X\n", 
                        pending_message.retries, pending_message.cmd_type, pending_message.msg_id);
            } else {
                // Max retries reached - give up
                DEBUG_PRINT("ERROR: Message failed after %d retries: CMD=0x%02X, ID=0x%02X\n", 
                        MAX_RETRANSMITS, pending_message.cmd_type, pending_message.msg_id);
                pending_message.in_use = false;

                missed_acks++;
                if (missed_acks >= MAX_MISSED_ACKS) {
                    DEBUG_PRINT("CRITICAL ERROR: %d consecutive messages lost, resetting communication state\n", missed_acks);
                    // Reset all pending messages
                    pending_message.in_use = false;
                    last_received_id = 0x00;
                    missed_acks = 0;
                }
            }
        }
    }
}

void uart_background_task() {
    process_timeouts();
    process_responses();
}

void queue_response(uint8_t cmd_type, const uint8_t *data, size_t data_length) {
    for (int i = 0; i < MAX_RESPONSES; i++) {
        if (!response_queue[i].ready) {
            // Found an empty slot
            response_queue[i].command = cmd_type;
            response_queue[i].data_length = data_length > RESPONSE_DATA_SIZE ? RESPONSE_DATA_SIZE : data_length;    // Cap data length if too long (shouldn't happen)
            memcpy(response_queue[i].data, data, response_queue[i].data_length);
            response_queue[i].ready = true;
            return;
        }
    }
    DEBUG_PRINT("ERROR: Response queue full, message dropped\n");
}

void process_responses(void) {
    for (int i = 0; i < MAX_RESPONSES; i++) {
        if (response_queue[i].ready) {
            if (pending_message.in_use) {
                continue;
            }
            send_command(response_queue[i].command, 
                             response_queue[i].data, 
                             response_queue[i].data_length);
            response_queue[i].ready = false;
        }
    }
}

void send_ack(uint8_t msg_id) {
    static pending_message_t ack_msg;
    ack_msg.in_use = false;
    ack_msg.msg_id = generate_msg_id();
    ack_msg.cmd_type = CMD_ACK;
    ack_msg.data_length = 1;
    ack_msg.data[0] = msg_id; // Echo back the sequence number being acknowledged
    send_uart_message(&ack_msg);
}

// UART RX interrupt handler
void on_uart_rx(void) {
    static uint8_t incoming_buffer[CMD_BUFFER_SIZE];
    static int incoming_buffer_index = 0;
    
    while (uart_is_readable(UART_ID)) {
        uint8_t c = uart_getc(UART_ID);
        
        if (c == 0) {
            if (incoming_buffer_index > 0) {
                uint8_t decoded[CMD_BUFFER_SIZE];
                size_t decoded_size = cobsDecode(incoming_buffer, incoming_buffer_index, decoded);
                
                DEBUG_PRINT("COBS frame rec'd (%d bytes), decoded to %d bytes\n", 
                       incoming_buffer_index, (int)decoded_size);
                
                if (decoded_size < 4) {  // CMD + ID + LEN + CRC minimum
                    DEBUG_PRINT("ERROR: Decoded message too short: %d bytes\n", (int)decoded_size);
                    continue;
                }

                uint8_t cmd_type = decoded[0];
                uint8_t msg_id = decoded[1];
                uint8_t data_length = decoded[2];
                if (decoded_size != data_length + 4) {  // CMD + ID + LEN + DATA + CRC
                    DEBUG_PRINT("ERROR: DMSG unexpected length: got %d, expected %d\n", 
                            (int)decoded_size, data_length + 4);
                    continue;
                }

                uint8_t received_crc = decoded[decoded_size - 1];
                uint8_t calculated_crc = calculate_crc8(decoded, decoded_size - 1);
                if (received_crc != calculated_crc) {
                    DEBUG_PRINT("ERROR: CRC8 mismatch! Received: 0x%02X, Calculated: 0x%02X\n", 
                            received_crc, calculated_crc);
                    continue;
                }

                if (msg_id == last_received_id) {
                    DEBUG_PRINT("Duplicate message ID=0x%02X, sending ACK only\n", msg_id);
                    queue_response(CMD_ACK, &msg_id, 1);
                    continue;
                }

                // New message, process it
                last_received_id = msg_id;
                DEBUG_PRINT("Command received: CMD=0x%02X, ID=0x%02X, Length=%d\n", 
                        cmd_type, msg_id, data_length);
                
                if (cmd_type != CMD_ACK) {
                    queue_response(CMD_ACK, &msg_id, 1);
                }
                
                // Process the command
                switch (cmd_type) {
                    case CMD_ACK:
                        if (data_length >= 1) {
                            uint8_t acked_id = decoded[3];  // First data byte
                            if (pending_message.in_use && pending_message.msg_id == acked_id) {
                                pending_message.in_use = false;
                                DEBUG_PRINT("MSG 0x%02X ACKed\n", acked_id);
                                missed_acks = 0;
                            }
                        }
                        break;
                    case CMD_PAUSE:
                        stepper_pause();
                        break;
                    case CMD_RESUME:
                        stepper_resume();
                        break;
                    case CMD_STOP:
                        stepper_set_enable(false);
                        break;
                    case CMD_MOVE_STATIC:
                        if (data_length >= 5) {
                            uint8_t axis = decoded[3];
                            int32_t position;
                            memcpy(&position, &decoded[4], sizeof(int32_t));
                            stepper_queue_static_move(axis, position);
                        } else {
                            DEBUG_PRINT("ERROR: CMD_MOVE_STATIC requires at least 5 bytes of data\n");
                        }
                        break;
                    case CMD_MOVE_TRACKING:
                        if (data_length >= 12) { // 3 floats (4 bytes each)
                            float x_rate, y_rate, z_rate;
                            
                            memcpy(&x_rate, &decoded[3], sizeof(float));
                            memcpy(&y_rate, &decoded[7], sizeof(float));
                            memcpy(&z_rate, &decoded[11], sizeof(float));
                            
                            DEBUG_PRINT("Received TRACK command: X=%.2f, Y=%.2f, Z=%.2f arcsec/sec\n", 
                                x_rate, y_rate, z_rate);
                                
                            stepper_start_tracking(x_rate, y_rate, z_rate);
                        } else {
                            DEBUG_PRINT("ERROR: TRACK command requires 12 data bytes\n");
                        }
                        break;
                    case CMD_GETPOS:
                        uint8_t response[12];
                        int32_t x = stepper_get_position_arcsec(AXIS_X);
                        int32_t y = stepper_get_position_arcsec(AXIS_Y);
                        int32_t z = stepper_get_position_arcsec(AXIS_Z);
                        memcpy(&response[0],  &x, sizeof(int32_t));
                        memcpy(&response[4],  &y, sizeof(int32_t));
                        memcpy(&response[8],  &z, sizeof(int32_t));

                        DEBUG_PRINT("Sending positions: X=%d, Y=%d, Z=%d (arcsec)\n", x, y, z);
                        queue_response(CMD_POSITION, response, 12);
                        break;
                }
            }
            
            incoming_buffer_index = 0;
        } else {
            if (incoming_buffer_index < CMD_BUFFER_SIZE - 1) {
                incoming_buffer[incoming_buffer_index++] = c;
            } else {
                DEBUG_PRINT("ERROR: COBS buffer overflow, resetting\n");
                incoming_buffer_index = 0;
            }
        }
    }
}