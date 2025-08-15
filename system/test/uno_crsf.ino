// CRSF constants
#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8
#define CRSF_ADDRESS_TRANSMITTER 0xEE
#define CRSF_FRAME_TYPE_RC_CHANNELS_PACKED 0x16
#define CRSF_MAX_CHANNELS 16

void setup() {
  Serial.begin(420000); // CRSF baud rate
}

void loop() {
  // RC values (range: 172–1811 for CRSF)
  uint16_t channels[CRSF_MAX_CHANNELS] = {
    172,  // Throttle - CRITICAL: Set to minimum (172) for arming
    1500, 1500, 1500, 1811,  // Roll, Pitch, Yaw, AUX1 (arm)
    172, 172, 172, 172, 172, 172, 172, 172, 172, 172, 172
  };

  sendCRSFRCFrame(channels);
  delay(20); // ~50Hz rate
}

void sendCRSFRCFrame(uint16_t *channels) {
  uint8_t frame[26];
  uint8_t i = 0;

  frame[i++] = CRSF_ADDRESS_FLIGHT_CONTROLLER; // Device address
  frame[i++] = 0x18; // Frame length (24 payload + 1 type + 1 CRC)
  frame[i++] = CRSF_FRAME_TYPE_RC_CHANNELS_PACKED;

  uint8_t bitOffset = 0;
  uint32_t bitBuffer = 0;
  uint8_t bitCount = 0;

  for (int ch = 0; ch < 16; ch++) {
    bitBuffer |= ((uint32_t)(channels[ch] & 0x07FF)) << bitCount;
    bitCount += 11;
    while (bitCount >= 8) {
      frame[i++] = bitBuffer & 0xFF;
      bitBuffer >>= 8;
      bitCount -= 8;
    }
  }
  if (bitCount > 0) {
    frame[i++] = bitBuffer & 0xFF;
  }

  // CRC
  uint8_t crc = 0;
  for (int j = 2; j < i; j++) {
    crc ^= frame[j];
  }
  frame[i++] = crc;

  Serial.write(frame, i);
}
