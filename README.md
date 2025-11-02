# BPpicoFW - Raspberry Pi Pico astrophotography mount controller
A highly configurable firmware for Raspberry Pi Pico that controls a 3-axis camera mount for astrophotography with stepper motors, temperature monitoring, and UART communication protocol. Intended to be used with the [StarSight PCB](https://github.com/tada66/BP_StarSight_PCB), 
however, could be easily adapted to different PCBs as the pin assignments are fully configurable. 

## Features
3-Axis Stepper Motor Control (X, Y, Z axes)\
Multi-axis simultaneous static positioning moves\
Tracking mode for a celestial object

## UART Communication Protocol
COBS (Consistent Overhead Byte Stuffing) encoding\
CRC8 error detection\
Message acknowledgment and retransmission\
DMA-based transmission for efficiency

## Temperature Monitoring
DS18B20 one-wire temperature sensor\
Automatic telemetry reporting every 10 seconds\
PWM Fan Control (fan is currently just set to 100% all the time, because I've found them to be pretty weak)

## Hardware Requirements
Raspberry Pi Pico\
[StarSight PCB](https://github.com/tada66/BP_StarSight_PCB)\
3x TMC2209 stepper motor drivers (A4988 will also work, but they are not recommended)\
4x Stepper motors (0.9° step angle recommended, but other angles configurable)\
DS18B20 temperature sensor (not strictly necessary as the TMC2209 has over temperature protection) 

## Command 
This project implements a robust custom UART communication protocol running at 9600 baud. This can be reconfigured to a faster speed in `UART.h` along with other variables to fine tune the communication protocol to your needs. 
But it is not recommended as the UART data can easily become corrupted at faster speed due to the proximity of the transmitting wires to the stepper motor wires.
### Command table
| Command name      | Command code  | Command direction | Data |     Description |
| ----------------- | ------------- | ----------------- | ---- | --------------- |
| CMD_ACK           | `0x01`        | -                 | -    | Acknowledgement |
| CMD_MOVE_STATIC   | `0x10`        | RPi->Pico         | `uint8_t` axis selection `uint32_t` position in arcseconds | Rotates the axis to the specified position from the reference point | 
| CMD_MOVE_TRACKING | `0x11`        | RPi->Pico         | `float32` X axis rate `float32` Y axis rate `float32` Z axis rate | Rotates each axis at a constant speed (speed specified in arcseconds/second) |
| CMD_PAUSE         | `0x12`        | RPi->Pico         | -    | Pauses all movement |
| CMD_RESUME        | `0x13`        | RPi->Pico         | -    | Resumes all movement and enables motors if they aren't enabled already |
| CMD_STOP          | `0x14`        | RPi->Pico         | -    | Disables motor drivers (applies power to the `EN` pin) |
| CMD_GETPOS        | `0x20`        | RPi->Pico         | `uint8_t` axis selection | Request for the current position of a specified axis |
| CMD_POSITION      | `0x21`        | Pico->RPi         | `uint8_t` axis selection `uint32_t` position in arcseconds | This will be changed to respond will the position of all axis |
| CMD_STATUS        | `0x22`        | Pico->RPi         | `float32` temperature | Telemetry data |
| CMD_ESTOPTRIG     | `0x30`        | Pico->RPi         | -    | Error: motor power cut, reference point lost |

### Command format
| Position      | Content       | Size     | Description                    |
| ------------- | ------------- | -------- | -----------------------------  |
| First byte    | Command code  | 1 byte   | For example `0x01` for CMD_ACK |
| Second byte   | MSG ID        | 1 byte   | Randomly generated ID for duplicate message detection, must be different from the last message, `0x00` is invalid |
| Third byte    | Data length   | 1 byte   | Data length of the message (message can be up to 255 bytes long) |
| Variable      | Message Data  | Variable |  |
| Last byte     | CRC8 checksum | 1 byte   | Control byte calculated across the whole message |

The message is the encoded with [COBS encoding](https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing) with `0x00` delimiter and sent on the UART0 interface.

## Architecture
**Core 0:** UART communication, temperature monitoring, main control loop\
**Core 1:** Stepper motor control with precise timing\
**DMA:** UART transmission for non-blocking communication
**Interrupts:** UART reception and DMA completion


