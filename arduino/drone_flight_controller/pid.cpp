#include "pid.h"

// Constructor for PID class
PID::PID(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd) {}

/**
 * Computes the PID control output based on the target and current values.
 * 
 * @param target The desired target value.
 * @param current The current measured value.
 * @param dt The time delta since the last computation.
 * @return The computed control output.
 */

float PID::compute(float target, float current, float dt) {
    float error = target - current; // Calculate error
    integral += error * dt; // Calculate integral of error
    float derivative = (error - lastError) / dt; // Calculate derivative of error
    lastError = error; // Update last error for next computation
    return kp * error + ki * integral + kd * derivative; // Compute and return PID output
}

