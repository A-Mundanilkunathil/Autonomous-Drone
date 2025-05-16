#pragma once

class PID {
    public:
        PID(float kp, float ki, float kd);
        float compute(float target, float current, float dt);
    private:
        float kp, ki, kd;
        float integral = 0;
        float lastError = 0;
}