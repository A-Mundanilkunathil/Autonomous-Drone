#pragma once
#include <Arduino_LSM9DS1.h>

void imuInit();
void readIMU(float &pitch, float &roll, float &yaw, float &gx, float &gy, float &gz);