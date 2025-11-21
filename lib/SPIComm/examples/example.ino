#include "SPIComm.h"

// Pin definitions
#define MOSI_PIN 35
#define MISO_PIN 34
#define SCLK_PIN 37
#define WAIT_PIN 39
#define RESET_PIN 38

// Create SPIComm instance
SPIComm spiComm(MOSI_PIN, MISO_PIN, SCLK_PIN, WAIT_PIN, RESET_PIN);

// Define your custom input frames (11 bytes each, CRC will be added automatically)
uint8_t myInputFrames[][11] = {
    {0x9, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00}, 
    {0xA, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}

};

const size_t numFrames = sizeof(myInputFrames) / sizeof(myInputFrames[0]);

// Define output buffer to receive responses (12 bytes per frame including CRC)
uint8_t myOutputBuffer[numFrames * 12];

void setup() {
  Serial.begin(115200);
  Serial.println("Main: Starting with custom buffers...");
  
  // Set custom input frames (pass as flat array)
  spiComm.setFrames((uint8_t*)myInputFrames, numFrames, 11);
  
  // Set output buffer to receive responses
  spiComm.setOutputBuffer(myOutputBuffer);
  
  // Initialize the SPI communication
  spiComm.begin();
}

void loop() {
  // Update the SPI communication
  spiComm.update();
  
  // You can access the received data in myOutputBuffer at any time
  // Each frame response is 12 bytes:
  // Frame 0: myOutputBuffer[0..11]
  // Frame 1: myOutputBuffer[12..23]
  // Frame 2: myOutputBuffer[24..35]
  // etc.
  
  // Example: Process received data
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 5000) {
    lastPrint = millis();
    
    Serial.println("\n=== Output Buffer Contents ===");
    for (size_t i = 0; i < numFrames; i++) {
      Serial.printf("Frame %d Response: ", i);
      for (size_t j = 0; j < 12; j++) {
        Serial.printf("0x%02X ", myOutputBuffer[i * 12 + j]);
      }
      Serial.println();
    }
    Serial.println("==============================\n");
  }
  
  // You can also modify the input frames on the fly
  // Just update myInputFrames[][] and the changes will be sent
  // in the next sequence
  
  // Example: Modify frame dynamically
  // myInputFrames[0][1] = random(0, 255);
}
