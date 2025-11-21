#include "SPIComm.h"

//#define DEBUG_PRINT  // Comment out this line to disable debug prints
#define SPIbus SPI2 

// Define the default frames data
const uint8_t SPIComm::_defaultFrames[][11] = {
    {0x9, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00}, 
    {0xA, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
};

const size_t SPIComm::_defaultNumFrames = sizeof(_defaultFrames) / sizeof(_defaultFrames[0]);

SPIComm::SPIComm(uint8_t mosiPin, uint8_t misoPin, uint8_t sclkPin, 
                 uint8_t waitPin, uint8_t resetPin)
  : _mosiPin(mosiPin),
    _misoPin(misoPin),
    _sclkPin(sclkPin),
    _waitPin(waitPin),
    _resetPin(resetPin),
    _spiSettings(1000000, LSBFIRST, SPI_MODE3),
    _currentState(STATE_IDLE),
    _stateTimer(0),
    _currentFrameIndex(0),
    _currentByteIndex(0),
    _sequenceActive(false),
    _initialized(false),
    _inputFrames(nullptr),
    _outputBuffer(nullptr),
    _numFrames(_defaultNumFrames),
    _frameSize(11),
    _useCustomFrames(false) {
}

void SPIComm::begin() {
  pinMode(_waitPin, INPUT);
  pinMode(_resetPin, OUTPUT);
  pinMode(_mosiPin, OUTPUT);
  pinMode(_misoPin, INPUT);
  pinMode(_sclkPin, OUTPUT);

  digitalWriteFast(_mosiPin, HIGH);
  digitalWriteFast(_sclkPin, HIGH);
  digitalWriteFast(_resetPin, HIGH);
  
#ifdef DEBUG_PRINT
  Serial.println("SPIComm: Initializing...");
#endif

  // Initialize SPI
  SPIbus.begin();
  
  // Start the reset sequence
  _currentState = STATE_RESET_LOW;
  _stateTimer = millis();
  _initialized = true;
  
#ifdef DEBUG_PRINT
  Serial.println("SPIComm: Setup Complete!");
#endif
}

void SPIComm::setFrames(uint8_t* frames, size_t numFrames, size_t frameSize) {
  _inputFrames = frames;
  _numFrames = numFrames;
  _frameSize = frameSize;
  _useCustomFrames = true;
}

void SPIComm::setOutputBuffer(uint8_t* outputBuffer) {
  _outputBuffer = outputBuffer;
}

size_t SPIComm::getCurrentFrameIndex() {
  return _currentFrameIndex;
}

size_t SPIComm::getNumFrames() {
  return _numFrames;
}

void SPIComm::update() {
  if (!_initialized) {
    return;
  }
  
  // State machine
  switch (_currentState) {
    case STATE_IDLE:
      // Auto-start the sequence
      startSequence();
      break;

    case STATE_RESET_LOW:
      digitalWrite(_resetPin, LOW);
      _stateTimer = millis();
      _currentState = STATE_RESET_HIGH;
      break;

    case STATE_RESET_HIGH:
      if (millis() - _stateTimer >= 100) {
        digitalWrite(_resetPin, HIGH);
        _stateTimer = millis();
        _currentState = STATE_WAIT_FOR_READY;
      }
      break;

    case STATE_WAIT_FOR_READY:
      if (millis() - _stateTimer >= 500) {
        if (digitalRead(_waitPin) == HIGH) {
          // Ready to start sequence
          startSequence();
        }
      }
      break;

    case STATE_BEGIN_TRANSACTION:
      if (_currentFrameIndex == 0 && _currentByteIndex == 0) {
#ifdef DEBUG_PRINT
        Serial.println("----------------Start sequence-----------------");
        Serial.println("");
#endif
      }
      
      // Prepare the current frame from custom or default frames
      if (_useCustomFrames && _inputFrames != nullptr) {
        memcpy(_currentFrame, _inputFrames + (_currentFrameIndex * _frameSize), _frameSize);
      } else {
        memcpy(_currentFrame, _defaultFrames[_currentFrameIndex], 11);
      }
      _currentFrame[11] = calculateCRC(_currentFrame, _frameSize);
      _currentByteIndex = 0;
      
      SPIbus.beginTransaction(_spiSettings);
      _currentState = STATE_WAIT_FOR_BYTE_READY;
      break;

    case STATE_WAIT_FOR_BYTE_READY:
      if (digitalReadFast(_waitPin) == HIGH) {
        _currentState = STATE_SEND_BYTE;
      }
      break;

    case STATE_SEND_BYTE:
      _rxBuffer[_currentByteIndex] = SPIbus.transfer(_currentFrame[_currentByteIndex]);
      _stateTimer = micros();
      _currentState = STATE_BYTE_DELAY;
      break;

    case STATE_BYTE_DELAY:
      if (micros() - _stateTimer >= BYTE_TIME) {
        _currentByteIndex++;
        
        if (_currentByteIndex >= 12) {
          // All bytes received - validate checksum
          bool checksumValid = validateChecksum(_rxBuffer, 12);
          
          // Store in output buffer if provided (overwrites previous frame)
          if (_outputBuffer != nullptr) {
            memcpy(_outputBuffer, _rxBuffer, 12);
          }
          
#ifdef DEBUG_PRINT
          // Print received frame with checksum status
          printFrame("Received Frame:", _rxBuffer, 12);
          if (checksumValid) {
            Serial.println("  ✓ Checksum valid");
          } else {
            Serial.println("  ✗ Checksum invalid!");
          }
#endif
          
          _currentState = STATE_END_TRANSACTION;
        } else {
          // More bytes to send
          _currentState = STATE_WAIT_FOR_BYTE_READY;
        }
      }
      break;

    case STATE_END_TRANSACTION:
      SPIbus.endTransaction();
      _stateTimer = micros();
      _currentState = STATE_FRAME_DELAY;
      break;

    case STATE_FRAME_DELAY:
      if (micros() - _stateTimer >= FRAME_TIME) {
        _currentFrameIndex++;
        
        if (_currentFrameIndex >= _numFrames) {
          // All frames sent
          _currentState = STATE_SEQUENCE_COMPLETE;
        } else {
          // More frames to send
          _currentState = STATE_BEGIN_TRANSACTION;
        }
      }
      break;

    case STATE_SEQUENCE_COMPLETE:
#ifdef DEBUG_PRINT
      Serial.println("");
      Serial.println("----------------End sequence-----------------");
#endif
      _currentFrameIndex = 0;
      // Automatically restart the sequence
      _currentState = STATE_BEGIN_TRANSACTION;
      break;
  }
}

void SPIComm::startSequence() {
  if (!_sequenceActive) {
    _sequenceActive = true;
    _currentFrameIndex = 0;
    _currentByteIndex = 0;
    _currentState = STATE_BEGIN_TRANSACTION;
  }
}

void SPIComm::stop() {
  _sequenceActive = false;
  _currentState = STATE_IDLE;
  SPIbus.endTransaction(); // Clean up if stopped mid-transaction
}

bool SPIComm::isActive() {
  return _sequenceActive;
}

uint8_t SPIComm::calculateCRC(const uint8_t* data, size_t length) {
  uint16_t crc = 0;
  for (size_t i = 0; i < length; i++) {
    crc += data[i];
  }
  return crc % 256;
}

bool SPIComm::validateChecksum(const uint8_t* data, size_t length) {
  if (length < 2) {
    return false;
  }
  
  // Calculate checksum of first 10 bytes
  uint16_t sum = 0;
  for (size_t i = 0; i < length - 2; i++) {
    sum += data[i];
  }
  
  // Extract received checksum (last two bytes, little-endian)
  uint16_t receivedChecksum = data[length - 1] | (data[length - 2] << 8);
  
  // Compare
  return sum == receivedChecksum;
}

void SPIComm::printFrame(const char* label, const uint8_t* data, size_t length) {
#ifdef DEBUG_PRINT
  Serial.print(label);
  for (size_t i = 0; i < length; i++) {
    Serial.printf(" 0x%02X", data[i]);
  }
  Serial.println();
#endif
}