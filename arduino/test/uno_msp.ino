#include <SoftwareSerial.h>

SoftwareSerial fcSerial(10, 11); // RX, TX

// RC channels: [Roll, Pitch, Throttle, Yaw, AUX1, AUX2, AUX3, AUX4]
uint16_t rc[8] = {1500, 1500, 1000, 1500, 2000, 1000, 1000, 1000}; // Arm using AUX1 = 2000

void setup() {
  fcSerial.begin(115200); // Match Betaflight MSP baud rate
  delay(1000);
}

void loop() {
  sendMSPSetRawRC(rc);
  delay(50); // Send at ~20Hz like real RX
}

/**
 * @brief Send an MSP SET_RAW_RC command to the flight controller.
 * @details Send an MSP SET_RAW_RC command to the flight controller with the given RC values.
 *          This function takes an array of 8 unsigned 16-bit integers, where each integer
 *          represents the value of an RC channel. Channels are ordered as follows:
 *          [Roll, Pitch, Throttle, Yaw, AUX1, AUX2, AUX3, AUX4].
 * @param rcData Array of 8 unsigned 16-bit integers representing the values of the RC channels.
 */
void sendMSPSetRawRC(uint16_t *rcData) {
  const uint8_t MSP_SET_RAW_RC = 200;
  uint8_t payload[16];

  for (int i = 0; i < 8; i++) {
    payload[2 * i]     = rcData[i] & 0xFF;
    payload[2 * i + 1] = rcData[i] >> 8;
  }

  // Send MSP packet
  fcSerial.write('$');
  fcSerial.write('M');
  fcSerial.write('<');

  uint8_t dataSize = 16;
  fcSerial.write(dataSize);

  uint8_t command = MSP_SET_RAW_RC;
  fcSerial.write(command);

  uint8_t checksum = dataSize ^ command;

  for (int i = 0; i < 16; i++) {
    fcSerial.write(payload[i]);
    checksum ^= payload[i];
  }

  fcSerial.write(checksum);
}
