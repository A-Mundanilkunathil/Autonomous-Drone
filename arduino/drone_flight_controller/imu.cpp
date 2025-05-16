#include "imu.h"
#include <math.h>

void imuInit() {
    // Initialize the IMU
    while (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        delay(1000);
    }
}

/**
 * @brief Read the IMU and calculate pitch, roll, and yaw angles.
 *
 * Reads the accelerometer and gyroscope data from the IMU, and calculates
 * the pitch, roll, and yaw angles from the acceleration data. The yaw angle
 * is assumed to come directly from the gyroscope.
 *
 * @param pitch Reference to a float variable to store the pitch angle.
 * @param roll Reference to a float variable to store the roll angle.
 * @param yaw Reference to a float variable to store the yaw angle.
 * @param gx Reference to a float variable to store the x-axis gyroscope reading.
 * @param gy Reference to a float variable to store the y-axis gyroscope reading.
 * @param gz Reference to a float variable to store the z-axis gyroscope reading.
 */
void readIMU(float &pitch, float &roll, float &yaw, float &gx, float &gy, float &gz) {
    float ax, ay, az; // Accelerometer readings

    // Read accelerometer and gyroscope data
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
        IMU.readAcceleration(ax, ay, az);
        IMU.readGyroscope(gx, gy, gz);

        // Calculate pitch, roll, and yaw angles
        pitch = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
        roll = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
        yaw = gz; // Assuming yaw is directly from gyroscope
    }
}