#include <Servo.h>

Servo esc1, esc2, esc3, esc4; // create servo objects to control the ESCs

void setup() {
    esc1.attach(6);
    esc2.attach(7);
    esc3.attach(8);
    esc4.attach(9);

    // Send MAX throttle to all ESCs
    esc1.writeMicroseconds(2000); // 2000 microseconds for maximum throttle
    esc2.writeMicroseconds(2000); // 2000 microseconds for maximum throttle
    esc3.writeMicroseconds(2000); // 2000 microseconds for maximum throttle
    esc4.writeMicroseconds(2000); // 2000 microseconds for maximum throttle
    delay(3000); // wait for 3 seconds to allow ESCs to initialize

    // Send MIN throttle to all ESCs
    esc1.writeMicroseconds(1000); // 1000 microseconds for minimum throttle
    esc2.writeMicroseconds(1000); // 1000 microseconds for minimum throttle 
    esc3.writeMicroseconds(1000); // 1000 microseconds for minimum throttle
    esc4.writeMicroseconds(1000); // 1000 microseconds for minimum throttle
    delay(3000); // wait for 3 seconds to allow ESCs to initialize

    // Calibrated
}

void loop() {

}