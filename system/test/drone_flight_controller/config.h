#pragma once

// Motor pins
#define ESC1_PIN 6
#define ESC2_PIN 9
#define ESC3_PIN 10
#define ESC4_PIN 11

// PID constants
#define KP 1.2 // Proportional gain
#define KI 0.01 // Integral gain
#define KD 0.6 // Derivative gain

// Base throttle
#define BASE_THROTTLE 1000
#define MAX_THROTTLE 2000
#define MIN_THROTTLE 700

// Loop timing
#define LOOP_DT 0.01 // Loop time in seconds (10 ms)