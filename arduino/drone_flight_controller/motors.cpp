#include "motors.h"
#include "config.h"

Servo esc1, esc2, esc3, esc4; // Create Servo objects for each ESC

/**
 * @brief Initialize the ESCs for the drone.
 * @details Initializes the ESCs for the drone by attaching them to the
 *          specified pins. Waits for 2 seconds to allow the ESCs to
 *          initialize.
 */
void motorsInit() {
    esc1.attach(ESC1_PIN); // Attach ESC to pin
    esc2.attach(ESC2_PIN); // Attach ESC to pin
    esc3.attach(ESC3_PIN); // Attach ESC to pin
    esc4.attach(ESC4_PIN); // Attach ESC to pin
    delay(2000); // Wait for ESCs to initialize
}

/**
 * @brief Set the speed of each ESC for the drone.
 * @details Given four speed values, set the speed of each ESC to the given
 *          value. The values are in microseconds, so the range is from
 *          MIN_THROTTLE to MAX_THROTTLE.
 * @param speed1 Speed for motor 1.
 * @param speed2 Speed for motor 2.
 * @param speed3 Speed for motor 3.
 * @param speed4 Speed for motor 4.
 */
void setMotorsSpeed(int speed1, int speed2, int speed3, int speed4) {
    esc1.writeMicroseconds(constrain(speed1, MIN_THROTTLE, MAX_THROTTLE)); // Set speed for ESC1
    esc2.writeMicroseconds(constrain(speed2, MIN_THROTTLE, MAX_THROTTLE)); // Set speed for ESC2
    esc3.writeMicroseconds(constrain(speed3, MIN_THROTTLE, MAX_THROTTLE)); // Set speed for ESC3
    esc4.writeMicroseconds(constrain(speed4, MIN_THROTTLE, MAX_THROTTLE)); // Set speed for ESC4
}