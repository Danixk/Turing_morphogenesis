// Type of motion
enum {STOP,LEFT,RIGHT,STRAIGHT};

/* 
 * A helper function for setting motor state.
 * Automatic spin-up of left/right motor, when necessary.
 * The standard way with spinup_motors() causes a small but
 * noticeable jump when a motor which is already running is spun up again.
 */
void smooth_set_motors(uint8_t ccw, uint8_t cw);

/*
 * Depending on the type of motion, the internal calibrated values for
 * the motors are sent through the previous smooth_set_motors function,
 * or zero if the motion required is STOP.
 */
void motion(uint8_t type);

