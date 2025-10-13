#ifndef STEPPER_H
#define STEPPER_H

#include "PIN_ASSIGNMENTS.h"
#include <stdio.h>  // For printf debugging
#include <math.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/sync.h"

#define X_STEPPER_GEAR_RATIO 28.5714285714f // 400:14
#define Y_STEPPER_GEAR_RATIO 23.5714285714f // 330:14
#define Z_STEPPER_GEAR_RATIO 30.0f          // 420:14

#define STEPS_PER_REV 400
#define MICROSTEPPING 16

enum {
    AXIS_X,
    AXIS_Y,
    AXIS_Z,
    NUM_AXES
};

typedef enum {
    STATIC_MOVE = 0,
    TRACKING_MOVE = 1
} stepper_command_type_t;

typedef struct {
    stepper_command_type_t type;
    bool valid;
    uint8_t axis;
    int32_t target_position;
} stepper_command_t;

typedef struct {
    bool tracking_active;
    float rates_arcsec_per_sec[NUM_AXES];  // Tracking rates in arcseconds per second for each axis
    uint32_t last_step_time[NUM_AXES];     // Time of the last step for each axis
} tracking_state_t;

void stepper_init_pins();
void stepper_init();
void stepper_core1_entry();
void stepper_set_enable(bool enable);
void stepper_pause();
void stepper_resume();

void stepper_queue_static_move(uint8_t axis, int32_t position);
int32_t stepper_get_position(uint8_t axis);
void stepper_start_tracking(float x_rate_arcsec, float y_rate_arcsec, float z_rate_arcsec);
int32_t stepper_get_position_arcsec(uint8_t axis);
int32_t arcseconds_to_steps(int32_t arcseconds, float gear_ratio);
int32_t steps_to_arcseconds(int32_t steps, float gear_ratio);

#endif // STEPPER_H