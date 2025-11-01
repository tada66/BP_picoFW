#include "STEPPER.h"
#include "hardware/timer.h"

// Global state variables
bool stepper_enabled = false;
volatile bool stepper_paused = false;

// Positional tracking
volatile int32_t x_position_steps = 0;
volatile int32_t y_position_steps = 0;
volatile int32_t z_position_steps = 0;

// Command structures
volatile stepper_command_t current_command = {STATIC_MOVE, false, 0, 0};
volatile tracking_state_t tracking_state = {false, {0.0f, 0.0f, 0.0f}, {0, 0, 0}};

// Helper functions for step/arcsecond conversions
int32_t arcseconds_to_steps(int32_t arcseconds, float gear_ratio) {
    // 1296000 = 360° * 60 * 60 (arcseconds in a full circle)
    float steps_per_arcsecond = ((float)STEPS_PER_REV * MICROSTEPPING * gear_ratio) / 1296000.0f;
    return (int32_t)(arcseconds * steps_per_arcsecond);
}

int32_t steps_to_arcseconds(int32_t steps, float gear_ratio) {
    float arcseconds_per_step = 1296000.0f / ((float)STEPS_PER_REV * MICROSTEPPING * gear_ratio);
    return (int32_t)(steps * arcseconds_per_step);
}

static inline uint get_step_pin(uint8_t axis) {
    switch (axis) {
        case AXIS_X: return X_STEP_PIN;
        case AXIS_Y: return Y_STEP_PIN;
        case AXIS_Z: return Z_STEP_PIN;
        default: return 0;
    }
}

static inline uint get_dir_pin(uint8_t axis) {
    switch (axis) {
        case AXIS_Z: return Z_DIR_PIN;
        case AXIS_Y: return Y_DIR_PIN;
        case AXIS_X: return X_DIR_PIN;
        default: return 0;
    }
}

static inline float get_gear_ratio(uint8_t axis) {
    switch (axis) {
        case AXIS_X: return X_STEPPER_GEAR_RATIO;
        case AXIS_Y: return Y_STEPPER_GEAR_RATIO;
        case AXIS_Z: return Z_STEPPER_GEAR_RATIO;
        default: return 1.0f;
    }
}

static inline volatile int32_t* get_position_ptr(uint8_t axis) {
    switch (axis) {
        case AXIS_X: return &x_position_steps;
        case AXIS_Y: return &y_position_steps;
        case AXIS_Z: return &z_position_steps;
        default: return NULL;
    }
}

void stepper_init_pins() {
    gpio_init(Y_STEP_PIN); gpio_set_dir(Y_STEP_PIN, GPIO_OUT);
    gpio_init(Y_DIR_PIN); gpio_set_dir(Y_DIR_PIN, GPIO_OUT);
    gpio_init(X_STEP_PIN); gpio_set_dir(X_STEP_PIN, GPIO_OUT);
    gpio_init(X_DIR_PIN); gpio_set_dir(X_DIR_PIN, GPIO_OUT);
    gpio_init(X_DIR_PIN_INV); gpio_set_dir(X_DIR_PIN_INV, GPIO_OUT);
    gpio_init(Z_STEP_PIN); gpio_set_dir(Z_STEP_PIN, GPIO_OUT);
    gpio_init(Z_DIR_PIN); gpio_set_dir(Z_DIR_PIN, GPIO_OUT);
    gpio_init(EN_SENSE_PIN); gpio_set_dir(EN_SENSE_PIN, GPIO_IN);
    gpio_init(EN_PIN); gpio_set_dir(EN_PIN, GPIO_OUT);    

    gpio_put(X_STEP_PIN, 0);
    gpio_put(X_DIR_PIN, 0);
    gpio_put(X_DIR_PIN_INV, 1); // REMEMBER: second X axis stepper must be inverted
    gpio_put(Y_STEP_PIN, 0);
    gpio_put(Y_DIR_PIN, 0);
    gpio_put(Z_STEP_PIN, 0);
    gpio_put(Z_DIR_PIN, 0);
    gpio_put(EN_PIN, 1); // Disable stepper driver initially
}

void stepper_init() {
    stepper_init_pins();
    multicore_launch_core1(stepper_core1_entry);
    DEBUG_PRINT("Stepper motor control initialized and launched on core 1\n");
}

void stepper_set_enable(bool enable) {
    gpio_put(EN_PIN, enable ? 0 : 1); // Active low
    stepper_enabled = enable;
    DEBUG_PRINT("Stepper motors %s\n", enable ? "enabled" : "disabled");
}

void stepper_pause() {
    stepper_paused = true;
    DEBUG_PRINT("Stepper motors paused\n");
}

void stepper_resume() {
    stepper_paused = false;
    DEBUG_PRINT("Stepper motors resumed\n");
    if(!stepper_enabled)
        stepper_set_enable(true);
}

void stepper_queue_static_move(uint8_t axis, int32_t position_arcsec) {
    if (!stepper_enabled) {
        DEBUG_PRINT("Stepper not enabled, cannot move!\n");
        return;
    }
    
    if (axis >= NUM_AXES) {
        DEBUG_PRINT("Invalid axis: %d\n", axis);
        return;
    }
    
    // Stop tracking mode if active
    if (tracking_state.tracking_active) {
        DEBUG_PRINT("Stopping tracking mode to execute static move\n");
        tracking_state.tracking_active = false;
    }
    
    // Replace any current command with the new one (don't wait)
    if (current_command.valid) {
        DEBUG_PRINT("Replacing current movement with new command\n");
        current_command.valid = false;
    }
    
    current_command.axis = axis;
    current_command.target_position = position_arcsec;
    current_command.type = STATIC_MOVE;
    current_command.valid = true;

    DEBUG_PRINT("Queued static move: Axis %d to %ld arcsec\n", axis, position_arcsec);
}

void stepper_start_tracking(float x_rate_arcsec, float y_rate_arcsec, float z_rate_arcsec) {
    if (!stepper_enabled) {
        DEBUG_PRINT("Stepper not enabled, cannot start tracking!\n");
        return;
    }
    
    // Stop any current movement command
    if (current_command.valid) {
        if(current_command.type == STATIC_MOVE) {
            DEBUG_PRINT("Stopping current static move to start tracking\n");
            sleep_ms(1);
        }
        current_command.valid = false;
    }
    
    // Set up tracking state
    tracking_state.tracking_active = true;
    tracking_state.rates_arcsec_per_sec[AXIS_X] = x_rate_arcsec;
    tracking_state.rates_arcsec_per_sec[AXIS_Y] = y_rate_arcsec;
    tracking_state.rates_arcsec_per_sec[AXIS_Z] = z_rate_arcsec;
    
    // Initialize last step times to current time
    uint32_t current_time = time_us_32();
    tracking_state.last_step_time[AXIS_X] = current_time;
    tracking_state.last_step_time[AXIS_Y] = current_time;
    tracking_state.last_step_time[AXIS_Z] = current_time;
    
    // Set direction pins based on rates
    gpio_put(X_DIR_PIN, x_rate_arcsec >= 0);
    gpio_put(X_DIR_PIN_INV, x_rate_arcsec < 0);  // Inverted
    gpio_put(Y_DIR_PIN, y_rate_arcsec >= 0);
    gpio_put(Z_DIR_PIN, z_rate_arcsec >= 0);
    
    DEBUG_PRINT("Started tracking mode: X=%0.2f, Y=%0.2f, Z=%0.2f arcsec/sec\n", 
           x_rate_arcsec, y_rate_arcsec, z_rate_arcsec);
}

void stepper_stop_tracking() {
    if (tracking_state.tracking_active) {
        tracking_state.tracking_active = false;
        DEBUG_PRINT("Tracking mode stopped\n");
    }
}

int32_t stepper_get_position(uint8_t axis) {
    if (axis >= NUM_AXES) {
        return 0;
    }
    
    volatile int32_t* pos_ptr = get_position_ptr(axis);
    return pos_ptr ? *pos_ptr : 0;
}

int32_t stepper_get_position_arcsec(uint8_t axis) {
    int32_t steps = stepper_get_position(axis);
    return steps_to_arcseconds(steps, get_gear_ratio(axis));
}

void stepper_core1_entry() {
    DEBUG_PRINT("Stepper core 1 started\n");
    
    // Updated timing for maximum TMC2209 performance
    static absolute_time_t last_step_time = {0};
    static absolute_time_t last_dir_change_time = {0};
    
    // Fast stepping: 1ms interval = 1kHz step rate (very fast but safe)
    static const uint32_t STEP_INTERVAL_MS = 1;  // 1ms = 1000 steps/sec
    
    // Direction setup time: 1μs (50x the required 20ns for safety)
    static const uint32_t DIR_SETUP_TIME_US = 1;

    
    while (true) {
        if (!stepper_enabled || stepper_paused) {
            sleep_ms(10);
            continue;
        }
        
        bool active_movement = false;
        absolute_time_t now = get_absolute_time();
        
        if (tracking_state.tracking_active) {
            active_movement = true;
            uint32_t current_time = time_us_32();
            
            for (uint8_t axis = 0; axis < NUM_AXES; axis++) {
                float rate = tracking_state.rates_arcsec_per_sec[axis];
                if (rate == 0.0f) continue;
                
                // Calculate interval between steps
                float gear_ratio = get_gear_ratio(axis);
                float steps_per_arcsec = ((float)STEPS_PER_REV * MICROSTEPPING * gear_ratio) / 1296000.0f;
                float steps_per_sec = fabsf(rate) * steps_per_arcsec;
                
                if (steps_per_sec > 0.0f) {
                    uint32_t step_interval_us = (uint32_t)(1000000.0f / steps_per_sec);
                    
                    // Check if it's time for a step
                    if ((current_time - tracking_state.last_step_time[axis]) >= step_interval_us) {
                        gpio_put(get_step_pin(axis), 1);
                        sleep_us(5);
                        gpio_put(get_step_pin(axis), 0);
                        
                        // Update position
                        volatile int32_t* pos_ptr = get_position_ptr(axis);
                        if (pos_ptr) {
                            if (rate > 0.0f) {
                                (*pos_ptr)++;
                            } else {
                                (*pos_ptr)--;
                            }
                        }

                        tracking_state.last_step_time[axis] = current_time;
                    }
                }
            }
        }
        // Process static movement commands
        else if (current_command.valid) {
            active_movement = true;
            uint8_t axis = current_command.axis;
            int32_t target;
            
            if (axis >= NUM_AXES) {
                current_command.valid = false;
                continue;
            }
            
            volatile int32_t* pos_ptr = get_position_ptr(axis);
            if (!pos_ptr) {
                current_command.valid = false;
                continue;
            }
            
            // Calculate target in steps based on command type
            if (current_command.type == STATIC_MOVE) {
                // Convert target position from arcseconds to steps
                target = arcseconds_to_steps(current_command.target_position, get_gear_ratio(axis));
            }
            
            // Calculate the difference (can be positive or negative)
            int32_t position_diff = target - *pos_ptr;
            
            // Determine direction based on the sign of the difference
            bool direction = position_diff >= 0;
            
            // Calculate absolute steps needed
            int32_t steps = direction ? position_diff : -position_diff;
            
            // Set direction with proper timing for TMC2209
            static bool last_direction[NUM_AXES] = {false, false, false};
            if (last_direction[axis] != direction) {
                gpio_put(get_dir_pin(axis), direction);
                if (axis == AXIS_X) {
                    gpio_put(X_DIR_PIN_INV, !direction);
                }
                last_direction[axis] = direction;
                last_dir_change_time = now;
            }
            
            // Check if it's time for the next step (non-blocking timing)
            bool direction_setup_complete = absolute_time_diff_us(last_dir_change_time, now) >= DIR_SETUP_TIME_US;
            bool step_interval_ready = absolute_time_diff_us(last_step_time, now) >= (STEP_INTERVAL_MS * 1000);
            
            if (steps > 0 && direction_setup_complete && step_interval_ready) {
                // Optimized step pulse timing for TMC2209
                gpio_put(get_step_pin(axis), 1);
                sleep_us(1);  // 1μs high pulse (10x the required 100ns)
                gpio_put(get_step_pin(axis), 0);
                // Implicit low time will be at least STEP_INTERVAL_MS duration
                
                // Update position in the correct direction
                if (direction) {
                    (*pos_ptr)++;
                } else {
                    (*pos_ptr)--;
                }
                
                // Update last step time
                last_step_time = now;
                
                // Print progress occasionally
                static int step_counter = 0;
                if (++step_counter % 100 == 0) {
                    DEBUG_PRINT("Axis %d: %ld steps remaining\n", axis, steps - 1);
                }
            } else if (steps == 0) {
                // Target reached
                current_command.valid = false;
                DEBUG_PRINT("Axis %d movement complete at position %ld steps\n", axis, *pos_ptr);
            }
        }
        
        // If no active movement, sleep briefly to avoid busy-waiting
        if (!active_movement) {
            sleep_ms(1);
        } else {
            // Small delay to prevent overwhelming the system
            sleep_us(100);
        }
    }
}