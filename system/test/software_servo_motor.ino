#include <SoftwareServo.h>

SoftwareServo esc1, esc2, esc3, esc4; // create servo objects to control the ESCs

void setup() {
    esc1.attach(6);
    esc2.attach(7);
    esc3.attach(8);
    esc4.attach(9);

    // Initialize ESCs 
    esc1.write(0);
    esc2.write(0);
    esc3.write(0);
    esc4.write(0);

    delay(3000); // Wait for ESCs to arm
}

void loop() {
    esc1.write(90); // Set speed to 1000 microseconds
    esc2.write(90); // Set speed to 1000 microseconds
    esc3.write(90); // Set speed to 1000 microseconds
    esc4.write(90); // Set speed to 1000 microseconds

    SoftwareServo::refresh(); // Refresh the servo to apply the speed
    delay(20); // Wait for 20 milliseconds before the next refresh
}