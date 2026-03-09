#include "STEPPER.h"

bool stepper_enabled = false;
volatile bool stepper_paused = true;
volatile bool celestial_tracking_slewing_finished = false;

volatile int32_t x_position_steps = 0;
volatile int32_t y_position_steps = 0;
volatile int32_t z_position_steps = 0;

// Multi-axis command structures - one command per axis
volatile stepper_command_t axis_commands[NUM_AXES] = {
    {STATIC_MOVE, false, AXIS_X, 0},
    {STATIC_MOVE, false, AXIS_Y, 0},
    {STATIC_MOVE, false, AXIS_Z, 0}
};

volatile tracking_state_t tracking_state = {false, {0.0f, 0.0f, 0.0f}, {0, 0, 0}};

// Celestial tracking state
volatile celestial_tracking_state_t celestial_state = {
    .active = false,
    .target_ra = 0.0f,
    .target_dec = 0.0f,
    .align_matrix = {1,0,0, 0,1,0, 0,0,1},  // Identity matrix
    .latitude = 0.0f,
    .ref_unix_time = 0,
    .ref_boot_time_us = 0
};

// Target positions for celestial tracking (computed each cycle)
static volatile int32_t celestial_target_arcsec[NUM_AXES] = {0, 0, 0};

int32_t arcseconds_to_steps(int32_t arcseconds, float gear_ratio) {
    // 1296000 = 360° * 60 * 60 (arcseconds in a full circle)
    float steps_per_arcsecond = ((float)STEPS_PER_REV * MICROSTEPPING * gear_ratio) / 1296000.0f;
    
    float exact_steps = arcseconds * steps_per_arcsecond;
    return (int32_t)(exact_steps >= 0 ? exact_steps + 0.5f : exact_steps - 0.5f);
}

int32_t steps_to_arcseconds(int32_t steps, float gear_ratio) {
    float arcseconds_per_step = 1296000.0f / ((float)STEPS_PER_REV * MICROSTEPPING * gear_ratio);
    
    float exact_arcseconds = steps * arcseconds_per_step;
    return (int32_t)(exact_arcseconds >= 0 ? exact_arcseconds + 0.5f : exact_arcseconds - 0.5f);
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
                                // this could be differently by actually letting these drivers not only share a step pin
                                // but also a direction pin, and swapping the wires of a single coil on the second motor
                                // ie swapping the red and blue wires, but since we already have a gpio for each drivers 
                                // dir pin we can just invert in software and have all the colors be the same way 
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

bool stepper_is_enabled(void) {
    return stepper_enabled;
}

bool stepper_is_paused(void) {
    return stepper_paused;
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
    
    // Stop any tracking modes
    if (tracking_state.tracking_active) {
        DEBUG_PRINT("Stopping rate tracking mode to execute static move\n");
        tracking_state.tracking_active = false;
    }
    if (celestial_state.active) {
        DEBUG_PRINT("Stopping celestial tracking to execute static move\n");
        celestial_state.active = false;
        celestial_tracking_slewing_finished = false;  // Clear status flag so status reports INACTIVE
    }
    
    // Set up command for this specific axis
    axis_commands[axis].axis = axis;
    axis_commands[axis].target_position = position_arcsec;
    axis_commands[axis].type = STATIC_MOVE;
    axis_commands[axis].valid = true;

    DEBUG_PRINT("Queued static move: Axis %d to %ld arcsec\n", axis, position_arcsec);
}

void stepper_queue_relative_move(uint8_t axis, int32_t offset_arcsec) {
    stepper_queue_static_move(axis, stepper_get_position_arcsec(axis) + offset_arcsec);
}

void stepper_stop_all_moves() {
    for (uint8_t axis = 0; axis < NUM_AXES; axis++) {
        axis_commands[axis].valid = false;
    }
    DEBUG_PRINT("All axis movements stopped\n");
}

void stepper_start_tracking(float x_rate_arcsec, float y_rate_arcsec, float z_rate_arcsec) {
    if (!stepper_enabled) {
        DEBUG_PRINT("Stepper not enabled, cannot start tracking!\n");
        return;
    }
    
    stepper_stop_all_moves();
    
    // Stop celestial tracking if active
    if (celestial_state.active) {
        celestial_state.active = false;
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
    
    // Direction GPIOs are set by the core 1 tracking loop
    // so that last_direction[] stays in sync
    
    DEBUG_PRINT("Started tracking mode: X=%0.2f, Y=%0.2f, Z=%0.2f arcsec/sec\n", 
           x_rate_arcsec, y_rate_arcsec, z_rate_arcsec);
}

void stepper_stop_tracking() {
    if (tracking_state.tracking_active) {
        tracking_state.tracking_active = false;
        DEBUG_PRINT("Tracking mode stopped\n");
    }
}


void stepper_start_celestial_tracking(float ra, float dec, const float* align_matrix, uint64_t ref_time, float latitude) {
    if (!stepper_enabled) {
        DEBUG_PRINT("Stepper not enabled, cannot start celestial tracking!\n");
        return;
    }
    
    stepper_stop_all_moves();
    tracking_state.tracking_active = false;
    celestial_tracking_slewing_finished = false;
    
    celestial_state.target_ra = ra;
    celestial_state.target_dec = dec;
    celestial_state.latitude = latitude; // Latitude is not used and will be removed in the future
    celestial_state.ref_unix_time = ref_time;
    celestial_state.ref_boot_time_us = time_us_32();
    
    for (int i = 0; i < 9; i++) {
        celestial_state.align_matrix[i] = align_matrix[i];
    }
    
    celestial_state.active = true;
    celestial_state.needs_unwrap_reset = true;
    
    DEBUG_PRINT("Started celestial tracking: RA=%.4fh, Dec=%.4f°, Lat=%.4f°\n", 
                ra, dec, latitude);
}

void stepper_stop_celestial_tracking(void) {
    if (celestial_state.active) {
        celestial_state.active = false;
        DEBUG_PRINT("Celestial tracking stopped\n");
    }
    celestial_tracking_slewing_finished = false;
}

bool stepper_is_celestial_tracking(void) {
    return celestial_tracking_slewing_finished && celestial_state.active;
}

static void compute_celestial_targets(void) {
    if (!celestial_state.active) return;
    
    uint32_t current_time_us = time_us_32();
    uint32_t elapsed_us;
    if (current_time_us >= celestial_state.ref_boot_time_us) {
        elapsed_us = current_time_us - celestial_state.ref_boot_time_us;
    } else {
        // Handle 32-bit wraparound (occurs every ~71 minutes)
        elapsed_us = (UINT32_MAX - celestial_state.ref_boot_time_us) + current_time_us + 1;
    }
    float elapsed_seconds = elapsed_us / 1000000.0f;
    
    // RA is in hours: 1h = 15° = 54000 arcsec
    float target_ra_arcsec = celestial_state.target_ra * 54000.0f;
    
    // RA drifts as Earth rotates (hour angle increases)
    float current_ra_arcsec = target_ra_arcsec - (elapsed_seconds * SIDEREAL_RATE_ARCSEC_PER_SEC);
    
    // Dec is in degrees: 1° = 3600 arcsec
    float target_dec_arcsec = celestial_state.target_dec * 3600.0f;
    
    // Step 3: Convert RA/Dec to unit vector (celestial sphere)
    float ra_rad = current_ra_arcsec * (M_PI / (180.0f * 3600.0f));
    float dec_rad = target_dec_arcsec * (M_PI / (180.0f * 3600.0f));
    
    float cos_dec = cosf(dec_rad);
    float sky_vector[3] = {
        cos_dec * cosf(ra_rad),   // X component
        cos_dec * sinf(ra_rad),   // Y component  
        sinf(dec_rad)             // Z component
    };
    
    // Step 4: Apply alignment matrix to transform sky -> mount coordinates
    // alignMatrix is row-major: [m00, m01, m02, m10, m11, m12, m20, m21, m22]
    // Matrix may be a pure rotation (det=1) or a general affine (det≠1) that
    // captures gear ratio errors and axis non-orthogonality.
    float mount_vector[3];
    mount_vector[0] = celestial_state.align_matrix[0]*sky_vector[0] + 
                      celestial_state.align_matrix[1]*sky_vector[1] + 
                      celestial_state.align_matrix[2]*sky_vector[2];
    mount_vector[1] = celestial_state.align_matrix[3]*sky_vector[0] + 
                      celestial_state.align_matrix[4]*sky_vector[1] + 
                      celestial_state.align_matrix[5]*sky_vector[2];
    mount_vector[2] = celestial_state.align_matrix[6]*sky_vector[0] + 
                      celestial_state.align_matrix[7]*sky_vector[1] + 
                      celestial_state.align_matrix[8]*sky_vector[2];
    
    // Normalize mount vector to unit length.
    // Essential for affine matrices (|A*sky| ≠ 1); safe no-op for rotations.
    // Without this, asinf() gets values outside [-1,1] → NaN.
    float mount_norm = sqrtf(mount_vector[0]*mount_vector[0] + 
                             mount_vector[1]*mount_vector[1] + 
                             mount_vector[2]*mount_vector[2]);
    if (mount_norm > 1e-6f) {
        mount_vector[0] /= mount_norm;
        mount_vector[1] /= mount_norm;
        mount_vector[2] /= mount_norm;
    }
    
    // Step 5: Convert unit vector to mount angles (in arcseconds)
    // X axis = tilt (altitude), Z axis = pan (azimuth)
    float mount_z_arcsec = atan2f(mount_vector[1], mount_vector[0]) * (180.0f * 3600.0f / M_PI);
    float mount_x_arcsec = asinf(mount_vector[2]) * (180.0f * 3600.0f / M_PI);
    
    // Step 6: Compute field rotation (Y axis)
    // Derive the angle between mount-zenith and celestial-pole directions,
    // both projected onto the plane perpendicular to the viewing direction.
    // This is the parallactic angle, computed purely from the alignment matrix.
    
    // Celestial pole (0,0,1) in mount frame = third column of alignment matrix.
    // For affine matrices this is not unit-length; normalize for correct projections.
    float pole_m[3] = {
        celestial_state.align_matrix[2],   // A[0,2]
        celestial_state.align_matrix[5],   // A[1,2]
        celestial_state.align_matrix[8]    // A[2,2]
    };
    float pole_norm = sqrtf(pole_m[0]*pole_m[0] + pole_m[1]*pole_m[1] + pole_m[2]*pole_m[2]);
    if (pole_norm > 1e-6f) {
        pole_m[0] /= pole_norm;
        pole_m[1] /= pole_norm;
        pole_m[2] /= pole_norm;
    }
    
    // Mount zenith = (0,0,1).  Project perpendicular to mount_vector:
    //   z_perp = z_hat - (z_hat · d) * d
    float dot_zd = mount_vector[2];
    float z_perp[3] = {
        -dot_zd * mount_vector[0],
        -dot_zd * mount_vector[1],
        1.0f - dot_zd * mount_vector[2]
    };
    
    // Project pole perpendicular to mount_vector:
    //   p_perp = pole_m - (pole_m · d) * d
    float dot_pd = pole_m[0]*mount_vector[0] + pole_m[1]*mount_vector[1] + pole_m[2]*mount_vector[2];
    float p_perp[3] = {
        pole_m[0] - dot_pd * mount_vector[0],
        pole_m[1] - dot_pd * mount_vector[1],
        pole_m[2] - dot_pd * mount_vector[2]
    };
    
    // Signed angle from z_perp to p_perp around mount_vector
    float dot_zp = z_perp[0]*p_perp[0] + z_perp[1]*p_perp[1] + z_perp[2]*p_perp[2];
    // (z_perp × p_perp) · d  gives  |z|*|p|*sin(θ)
    float cross_d = (z_perp[1]*p_perp[2] - z_perp[2]*p_perp[1]) * mount_vector[0]
                  + (z_perp[2]*p_perp[0] - z_perp[0]*p_perp[2]) * mount_vector[1]
                  + (z_perp[0]*p_perp[1] - z_perp[1]*p_perp[0]) * mount_vector[2];
    
    float field_rotation_rad = atan2f(cross_d, dot_zp);
    float mount_y_arcsec = field_rotation_rad * (180.0f * 3600.0f / M_PI);
    
    // Step 7: Unwrap Z and Y angles for continuity.
    // atan2 returns values in (-180°,180°) = (-648000,648000) arcsec.
    // Without unwrapping, crossing ±180° causes a 360° jump in the target,
    // making the motor reverse direction and slew a full turn.
    // Fix: keep targets continuous by adjusting to the nearest equivalent angle.
    static int32_t prev_z = 0;
    static int32_t prev_y = 0;
    
    int32_t new_z = (int32_t)mount_z_arcsec;
    int32_t new_y = (int32_t)mount_y_arcsec;
    
    if (celestial_state.needs_unwrap_reset) {
        celestial_state.needs_unwrap_reset = false;
    } else {
        // Unwrap Z: find equivalent angle closest to previous target
        int32_t dz = new_z - prev_z;
        while (dz > 648000)  dz -= 1296000;
        while (dz < -648000) dz += 1296000;
        new_z = prev_z + dz;
        
        // Unwrap Y: same
        int32_t dy = new_y - prev_y;
        while (dy > 648000)  dy -= 1296000;
        while (dy < -648000) dy += 1296000;
        new_y = prev_y + dy;
    }
    
    prev_z = new_z;
    prev_y = new_y;
    
    // Store computed targets
    celestial_target_arcsec[AXIS_X] = (int32_t)mount_x_arcsec;
    celestial_target_arcsec[AXIS_Z] = new_z;
    celestial_target_arcsec[AXIS_Y] = new_y;
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

// Trapezoidal velocity ramp: linearly interpolates step interval between
// RAMP_MAX_INTERVAL_US (slow start) and RAMP_MIN_INTERVAL_US (full speed).
// Uses min(steps_done, steps_remaining) so the motor decelerates symmetrically.
static inline uint32_t compute_ramp_interval(uint32_t steps_done, uint32_t steps_remaining) {
    uint32_t ramp_pos = steps_done < steps_remaining ? steps_done : steps_remaining;
    if (ramp_pos >= RAMP_STEPS) return RAMP_MIN_INTERVAL_US;
    return RAMP_MAX_INTERVAL_US -
           (uint32_t)((uint64_t)(RAMP_MAX_INTERVAL_US - RAMP_MIN_INTERVAL_US) * ramp_pos / RAMP_STEPS);
}

void stepper_core1_entry() {
    DEBUG_PRINT("Stepper core 1 started\n");
    
    // Per-axis timing variables for multi-axis support
    static absolute_time_t last_step_time[NUM_AXES] = {0};
    static absolute_time_t last_dir_change_time[NUM_AXES] = {0};

    // Direction tracking per axis
    static bool last_direction[NUM_AXES] = {false, false, false};

    // Ramp state per axis
    static uint32_t ramp_steps_done[NUM_AXES] = {0};
    static bool ramp_was_moving[NUM_AXES] = {false, false, false};
    
    while (true) {
        if (!stepper_enabled || stepper_paused) {
            sleep_ms(IDLE_SLEEP_MS);
            continue;
        }
        
        bool active_movement = false;
        absolute_time_t now = get_absolute_time();
        
        // Celestial tracking mode - autonomous position tracking
        if (celestial_state.active) {
            active_movement = true;
            
            // Compute target positions based on current time
            compute_celestial_targets();
            
            bool all_axes_at_target = true;
            // Move each axis towards its computed target
            for (uint8_t axis = 0; axis < NUM_AXES; axis++) {
                volatile int32_t* pos_ptr = get_position_ptr(axis);
                if (!pos_ptr) continue;
                
                // Get target in steps
                int32_t target_steps = arcseconds_to_steps(celestial_target_arcsec[axis], get_gear_ratio(axis));
                int32_t position_diff = target_steps - *pos_ptr;
                
                if (position_diff == 0){
                    ramp_was_moving[axis] = false;
                    continue;
                }
                // Less than 3 steps difference means were close enough to be tracking instead of just chasing the object
                if (position_diff > 3 || position_diff < -3) {
                    all_axes_at_target = false;
                }
                
                bool direction = position_diff > 0;
                uint32_t steps_remaining = direction ? (uint32_t)position_diff : (uint32_t)(-position_diff);
                
                // Update direction if changed — also reset ramp
                if (last_direction[axis] != direction) {
                    gpio_put(get_dir_pin(axis), direction);
                    if (axis == AXIS_X) {
                        gpio_put(X_DIR_PIN_INV, !direction);
                    }
                    last_direction[axis] = direction;
                    last_dir_change_time[axis] = now;
                    ramp_steps_done[axis] = 0;
                }
                // Reset ramp when axis starts from stopped
                if (!ramp_was_moving[axis]) {
                    ramp_steps_done[axis] = 0;
                    ramp_was_moving[axis] = true;
                }
                
                // Variable-speed step interval: ramp up from start, ramp down near target
                uint32_t step_interval = compute_ramp_interval(ramp_steps_done[axis], steps_remaining);
                bool direction_setup_complete = absolute_time_diff_us(last_dir_change_time[axis], now) >= DIR_SETUP_TIME_US;
                bool step_interval_ready = absolute_time_diff_us(last_step_time[axis], now) >= step_interval;
                
                if (direction_setup_complete && step_interval_ready) {
                    gpio_put(get_step_pin(axis), 1);
                    sleep_us(STEP_PULSE_WIDTH_US);
                    gpio_put(get_step_pin(axis), 0);
                    
                    if (direction) {
                        (*pos_ptr)++;
                    } else {
                        (*pos_ptr)--;
                    }
                    
                    last_step_time[axis] = now;
                    ramp_steps_done[axis]++;
                }
            }
            if (all_axes_at_target) {
                celestial_tracking_slewing_finished = true;
            }
        }
        // Rate-based tracking mode
        else if (tracking_state.tracking_active) {
            active_movement = true;
            uint32_t current_time = time_us_32();
            
            for (uint8_t axis = 0; axis < NUM_AXES; axis++) {
                float rate = tracking_state.rates_arcsec_per_sec[axis];
                if (rate == 0.0f) continue;
                
                // Set direction based on rate sign, keeping last_direction in sync
                bool direction = (rate > 0.0f);
                if (last_direction[axis] != direction) {
                    gpio_put(get_dir_pin(axis), direction);
                    if (axis == AXIS_X) {
                        gpio_put(X_DIR_PIN_INV, !direction);
                    }
                    last_direction[axis] = direction;
                    last_dir_change_time[axis] = now;
                }
                
                // Calculate interval between steps
                float gear_ratio = get_gear_ratio(axis);
                float steps_per_arcsec = ((float)STEPS_PER_REV * MICROSTEPPING * gear_ratio) / 1296000.0f;
                float steps_per_sec = fabsf(rate) * steps_per_arcsec;
                
                if (steps_per_sec > 0.0f) {
                    uint32_t step_interval_us = (uint32_t)(1000000.0f / steps_per_sec);
                    
                    bool direction_setup_complete = absolute_time_diff_us(last_dir_change_time[axis], now) >= DIR_SETUP_TIME_US;
                    
                    // Check if it's time for a step
                    if (direction_setup_complete && (current_time - tracking_state.last_step_time[axis]) >= step_interval_us) {
                        gpio_put(get_step_pin(axis), 1);
                        sleep_us(TRACKING_STEP_PULSE_US);  // Use defined constant
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
        // Process static movement commands for all axes simultaneously
        else {
            // Process each axis independently
            for (uint8_t axis = 0; axis < NUM_AXES; axis++) {
                if (!axis_commands[axis].valid) continue;
                
                active_movement = true;
                
                volatile int32_t* pos_ptr = get_position_ptr(axis);
                if (!pos_ptr) {
                    axis_commands[axis].valid = false;
                    continue;
                }
                
                // Calculate target in steps
                int32_t target = arcseconds_to_steps(axis_commands[axis].target_position, get_gear_ratio(axis));
                
                // Calculate the difference (can be positive or negative)
                int32_t position_diff = target - *pos_ptr;
                
                // Determine direction based on the sign of the difference
                bool direction = position_diff >= 0;
                
                // Calculate absolute steps needed
                int32_t steps = direction ? position_diff : -position_diff;
                
                if (last_direction[axis] != direction) {
                    gpio_put(get_dir_pin(axis), direction);
                    
                    if (axis == AXIS_X) {
                        gpio_put(X_DIR_PIN_INV, !direction);
                    }
                    
                    last_direction[axis] = direction;
                    last_dir_change_time[axis] = now;
                    ramp_steps_done[axis] = 0;
                }
                // Reset ramp when axis starts from stopped
                if (!ramp_was_moving[axis]) {
                    ramp_steps_done[axis] = 0;
                    ramp_was_moving[axis] = true;
                }
                
                // Variable-speed step interval: ramp up from start, ramp down near target
                uint32_t step_interval = compute_ramp_interval(ramp_steps_done[axis], (uint32_t)steps);
                bool direction_setup_complete = absolute_time_diff_us(last_dir_change_time[axis], now) >= DIR_SETUP_TIME_US;
                bool step_interval_ready = absolute_time_diff_us(last_step_time[axis], now) >= step_interval;
                
                if (steps > 0 && direction_setup_complete && step_interval_ready) {
                    gpio_put(get_step_pin(axis), 1);
                    sleep_us(STEP_PULSE_WIDTH_US);
                    gpio_put(get_step_pin(axis), 0);
                    
                    // Update position in the correct direction
                    if (direction) {
                        (*pos_ptr)++;
                    } else {
                        (*pos_ptr)--;
                    }
                    
                    // Update last step time for this axis
                    last_step_time[axis] = now;
                    ramp_steps_done[axis]++;
                    
                    static int step_counter[NUM_AXES] = {0};
                    if (++step_counter[axis] % 1000 == 0) {
                        DEBUG_PRINT("Axis %d: %ld steps remaining\n", axis, steps - 1);
                    }
                } else if (steps == 0) {
                    // Target reached for this axis
                    axis_commands[axis].valid = false;
                    ramp_was_moving[axis] = false;
                    DEBUG_PRINT("Axis %d movement complete at position %ld steps\n", axis, *pos_ptr);
                }
            }
        }
        
        if (!active_movement) {
            sleep_ms(INACTIVE_SLEEP_MS);
        } else {
            sleep_us(ACTIVE_SLEEP_US);
        }
    }
}