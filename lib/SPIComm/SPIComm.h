#ifndef SPICOMM_H
#define SPICOMM_H

#include <Arduino.h>
#include <SPI.h>

#define FRAME_TIME 3500
#define BYTE_TIME 200 

class SPIComm {
public:
  // Constructor
  SPIComm(uint8_t mosiPin, uint8_t misoPin, uint8_t sclkPin, 
          uint8_t waitPin, uint8_t resetPin);
  
  // Initialize the SPI communication
  void begin();
  
  // Set custom frame buffers (each frame should be 11 bytes, CRC will be added automatically)
  void setFrames(uint8_t* frames, size_t numFrames, size_t frameSize = 11);
  
  // Set output buffer to receive data (should be at least 12 bytes)
  // The same buffer is updated with each frame response
  void setOutputBuffer(uint8_t* outputBuffer);
  
  // Update function - call this in your main loop
  void update();
  
  // Check if sequence is currently active
  bool isActive();
  
  // Start a new sequence manually (optional, it auto-starts)
  void startSequence();
  
  // Stop the sequence
  void stop();
  
  // Get the current frame index being processed
  size_t getCurrentFrameIndex();
  
  // Get total number of frames
  size_t getNumFrames();

private:
  // Pin definitions
  uint8_t _mosiPin;
  uint8_t _misoPin;
  uint8_t _sclkPin;
  uint8_t _waitPin;
  uint8_t _resetPin;
  
  // State machine states
  enum State {
    STATE_IDLE,
    STATE_RESET_LOW,
    STATE_RESET_HIGH,
    STATE_WAIT_FOR_READY,
    STATE_BEGIN_TRANSACTION,
    STATE_WAIT_FOR_BYTE_READY,
    STATE_SEND_BYTE,
    STATE_BYTE_DELAY,
    STATE_END_TRANSACTION,
    STATE_FRAME_DELAY,
    STATE_SEQUENCE_COMPLETE
  };
  
  // State machine variables
  State _currentState;
  unsigned long _stateTimer;
  size_t _currentFrameIndex;
  size_t _currentByteIndex;
  uint8_t _currentFrame[12];
  uint8_t _rxBuffer[12];
  bool _sequenceActive;
  bool _initialized;
  
  // Frame buffer pointers (for custom buffers)
  uint8_t* _inputFrames;
  uint8_t* _outputBuffer;
  size_t _numFrames;
  size_t _frameSize;
  bool _useCustomFrames;
  
  // Default frames data (used if no custom frames set)
  static const uint8_t _defaultFrames[][11];
  static const size_t _defaultNumFrames;
  
  // SPI Settings
  SPISettings _spiSettings;
  
  // Helper functions
  uint8_t calculateCRC(const uint8_t* data, size_t length);
  bool validateChecksum(const uint8_t* data, size_t length);
  void printFrame(const char* label, const uint8_t* data, size_t length);
};

#endif // SPICOMM_H