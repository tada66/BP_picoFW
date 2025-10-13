#ifndef BPPICOFW_H
#define BPPICOFW_H

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "DS18B20.h"
#include "PIN_ASSIGNMENTS.h"
#include "UART.h"
#include "STEPPER.h"

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"

void fan_set_speed(float duty_percent);
int main();

#endif // BPPICOFW_H