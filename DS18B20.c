#include "DS18B20.h"
#include "PIN_ASSIGNMENTS.h"

// --- 1-Wire primitives ---
static void onewire_write_bit(bool bit) {
    gpio_put(TEMP_SENSE_PIN, 0);
    sleep_us(bit ? 6 : 60);
    gpio_put(TEMP_SENSE_PIN, 1);
    if (bit) sleep_us(64);
    else sleep_us(10);
}

static bool onewire_read_bit(void) {
    bool bit;
    gpio_put(TEMP_SENSE_PIN, 0);
    sleep_us(6);
    gpio_put(TEMP_SENSE_PIN, 1);
    sleep_us(9);
    bit = gpio_get(TEMP_SENSE_PIN);
    sleep_us(55);
    return bit;
}

static void onewire_write_byte(uint8_t data) {
    for (int i=0; i<8; i++) {
        onewire_write_bit(data & 1);
        data >>= 1;
    }
}

static uint8_t onewire_read_byte(void) {
    uint8_t data = 0;
    for (int i=0; i<8; i++) {
        if (onewire_read_bit()) data |= (1 << i);
    }
    return data;
}

static bool onewire_reset(void) {
    gpio_put(TEMP_SENSE_PIN, 0);
    sleep_us(480);
    gpio_put(TEMP_SENSE_PIN, 1);
    sleep_us(70);
    bool presence = !gpio_get(TEMP_SENSE_PIN);
    sleep_us(410);
    return presence;
}

// --- DS18B20 specific ---
float ds18b20_read_temp(void) {
    if (!onewire_reset()) return -1000.0f; // no sensor

    onewire_write_byte(0xCC); // Skip ROM
    onewire_write_byte(0x44); // Convert T
    sleep_ms(750); // wait for conversion

    onewire_reset();
    onewire_write_byte(0xCC); // Skip ROM
    onewire_write_byte(0xBE); // Read Scratchpad

    uint8_t lsb = onewire_read_byte();
    uint8_t msb = onewire_read_byte();
    int16_t raw = (msb << 8) | lsb;

    return raw / 16.0f;
}