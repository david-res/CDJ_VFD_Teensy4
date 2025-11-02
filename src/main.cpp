#include "Arduino.h"
//#include "RemoteDisplay.h"
#include "lvgl.h"
#include "SPI.h"
//#include "SPISlave_T4.h"
#include "QuadEncoder.h"
#include "T4_DMA_SPI_SLAVE.h"
#include "ST7796s_t4_mm.h"
#include "Bounce2.h"
#include "globals.h"

#define BYTE_BUF_SIZE 512
uint8_t spi_slave_TXbuf[BYTE_BUF_SIZE];
uint8_t spi_slave_RXbuf[BYTE_BUF_SIZE];
#define slaveSPI SPI_SLAVE1

//SPISlave_T4<&SPI, SPI_8_BITS> slaveSPI; // Slave settings for the main control unit to control this Teensy. It will be sening and reciving masterRxBuffer and masterTxBuffer
//Slave is on SPI1
SPISettings panelSPI(2000000, LSBFIRST, SPI_MODE3); // Master settings for controlling the CDJ1000 MK1 control panel. It needs bus speed set at 1Mhz, SPI mode3 and LSB first.
//CDJ Panel is on SPI2

ST7796s_t4_mm lcd = ST7796s_t4_mm(ST7796_DC, ST7796_CS, ST7796_RST);

QuadEncoder jogEncoder(0, JOG0, JOG1);

IntervalTimer dataTimer;
//CDJ PANEL VARS AND SETTINGS
// Define constants
const uint8_t FRAME_SIZE = 12;
const uint16_t FRAME_DELAY_US = 3000;
const uint16_t BYTE_DELAY_US = 300;

// Define variables for non-blocking operation
unsigned long frameStartTime = 0;
unsigned long byteStartTime = 0;
size_t currentFrameIndex = 0;
size_t currentByteIndex = 0;
uint8_t currentFrame[FRAME_SIZE];
uint8_t rxBuffer[FRAME_SIZE];
bool isTransactionActive = false;
bool waitingForWaitPin = false;


//JOG+TOUCH VARS AND SETTINGS
volatile uint32_t lastPulseMicros = 0;
volatile uint16_t jogVelocity = 65535;  // time between pulses in µs (clamped)
volatile int32_t jogDirection = 0;
volatile bool jogMoving = false;

volatile bool touchLatched = false;
bool touchActive = false;
Bounce touchDebouncer = Bounce();

uint16_t adin = 0;
uint16_t adct = 0;
float pitchSlider = 0.0;



    

lv_display_t* disp; // pointer to lvgl display object

//RemoteDisplay remoteDisplay;

lv_obj_t * screen;
void createScreen();
void createVFD();

lv_obj_t * vfd_scale;
lv_obj_t * play_pos_obj;


lv_obj_t * VDF_TOUCH_IND;
lv_obj_t * VDF_BG;
lv_obj_t * VDF_CUE_MEM;
lv_obj_t * VDF_VINYL_IND;


void dataTimerISR() {
  // This function can be used for periodic tasks if needed
  adin = analogRead(ADIN);
  adct = analogRead(ADCT);
  /*
  [byte 10][--] = PITCH MSB (0-1023)
  [byte 11][--] = PITCH LSB (0-1023)
  [byte 12][--] = PITCH CENTER MSB (0-1023)
  [byte 13][--] = PITCH CENTER LSB (0-1023)
  [byte 14][--] = JOG POSITION MSB (0-65535)
  [byte 15][--] = JOG POSITION LSB (0-65535)
  [byte 16][bit 0] = JOG direction (1 = forward, 0 = reverse)
  [byte 16][bit 1] = JOG touch enable (1 = enabled, 0 = disabled)
  */
  for(int i = 0; i < 10; i++) {
    masterTxBuffer[i] = panelRxBuffer[i]; // Copy the first 10 bytes from panelRxBuffer to masterTxBuffer
  }

  masterTxBuffer[10] = (adin >> 8) & 0x03; // PITCH MSB
  masterTxBuffer[11] = adin & 0xFF;        // PITCH LSB
  masterTxBuffer[12] = (adct >> 8) & 0x03; // PITCH CENTER MSB
  masterTxBuffer[13] = adct & 0xFF;        // PITCH CENTER LSB
  masterTxBuffer[14] = (jogVelocity >> 8) & 0xFF; // JOG POSITION MSB
  masterTxBuffer[15] = jogVelocity & 0xFF;        // JOG POSITION LSB
  masterTxBuffer[16] = (jogMoving ? 1 : 0);       // JOG direction
  masterTxBuffer[16] |= (touchActive ? 1 : 0) << 1; // JOG touch enable

}


void jogISR() {
  uint32_t now = micros();
  uint32_t delta = now - lastPulseMicros;
  lastPulseMicros = now;

  if (delta > 65535) {
    jogVelocity = 65535;
  } else {
    jogVelocity = (uint16_t)delta;
  }

  int dir = digitalReadFast(JOG1) ? 1 : -1;
  jogDirection += dir;
  jogMoving = true;

  if (touchActive) {
    touchLatched = true;
  }
}

void lcdCallback(){

  lv_disp_flush_ready(disp);

}

/** LVGL Callback to draw on the screen */
void my_disp_flush(lv_display_t* disp_, const lv_area_t* area, uint8_t* px_map){
      arm_dcache_flush_delete((uint16_t*)fb1, sizeof(fb1)); // always flush cache after writing to DMAMEM variable that will be accessed by DMA
      arm_dcache_flush_delete((uint16_t*)fb2, sizeof(fb2)); // always flush cache after writing to DMAMEM variable that will be accessed by DMA
      lcd.pushPixels16bitDMA((uint16_t*)px_map, area->x1, area->y1, area->x2, area->y2);
      //Serial.println("Flush");
}



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //while (!Serial && millis() < 4000) {
    // Wait for Serial
  //}
  Serial.println("Begin Setup");

  //slaveSPI.begin(SPI_MODE0, MSBFIRST, MODE_4WIRE);
  //slaveSPI.auto_repeat = 1;
  //slaveSPI.print_pin_use();
  //slaveSPI.prepare_for_slave_transfer(spi_slave_TXbuf, spi_slave_RXbuf, 27);
  pinMode(CDJ_WAIT, INPUT);
  pinMode(CDJ_RST, OUTPUT);

  digitalWriteFast(CDJ_RST, LOW);
  delay(100);
  digitalWriteFast(CDJ_RST, HIGH);
  SPI2.begin();

  //LCD Backlight control
 


  //JOG Wheek pin setups
  pinMode(JOG0, INPUT);
  pinMode(JOG1, INPUT);
  //pinMode(JOG_TOUCH, INPUT);
  attachInterrupt(digitalPinToInterrupt(JOG0), jogISR, RISING);

  //JOG Touch sensor Debounce setup
  touchDebouncer.attach(JOG_TOUCH, INPUT_PULLUP);
  touchDebouncer.interval(10);  // debounce in ms

  analogReadResolution(10);


  //LCD Start
  lcd.begin(30);
  lcd.onCompleteCB(lcdCallback);
  memset((uint16_t*)fb1, 0xFFFF, sizeof(fb1));
  pinMode(LCD_BL, OUTPUT);
  analogWrite(LCD_BL, 200);

  //lcd.pushPixels16bit(fb1,0,0,319,319);

  delay(50);


  //LVGL Init
  lv_init();
  disp = lv_display_create(320, 320);
  lv_display_set_flush_cb(disp, my_disp_flush);
  lv_display_set_buffers(disp, fb1, fb2, 320*320 * sizeof(int16_t), LV_DISPLAY_RENDER_MODE_PARTIAL);

  lv_tick_set_cb(millis);


  createScreen();
  createVFD();
  Serial.println("Screen & VFD created");  
}




unsigned long previousMillis0 = 0; 
unsigned long previousMillis1 = 0; 

void nonBlockingSPITransfer();


void loop() {

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis0 >= 3) {
    // Save the last time lv_timer_handler was called
    previousMillis0 = currentMillis;

    // Call lv_timer_handler
    lv_timer_handler();   
  }
  

  nonBlockingSPITransfer();

  
  if (currentMillis - previousMillis1 >=20) {
    // Save the last time lv_timer_handler was called
    previousMillis1 = currentMillis;
    updateVDF();
  }


  touchDebouncer.update();
  touchActive = touchDebouncer.read() == LOW;
  //Serial.printf ("Touch Active: %d \n", touchActive);

  static uint32_t lastCheck = 0;
  if (millis() - lastCheck > 10) {
    lastCheck = millis();

    static int32_t lastPosition = 0;
    int32_t currentPos = jogDirection;
    uint32_t timeSinceLastPulse = micros() - lastPulseMicros;

    if (currentPos == lastPosition && timeSinceLastPulse > UINT16_MAX) {
      jogMoving = false;

      if (touchLatched) {
       // Serial.println("Touch released after jog stop");
        touchLatched = false;
      }
    }
    lastPosition = currentPos;


    static int32_t lastPrintedPos = 0;
    int32_t pos = jogDirection;
    if (pos != lastPrintedPos) {
      uint16_t v = jogVelocity;
      //Serial.printf("JogPos: %ld, jogVelocity (Δt): %u µs, TouchLatched: %d\n",pos, v, touchLatched);
      lastPrintedPos = pos;
    }
  }

  
  if (touchLatched || touchActive){
    masterTxBuffer[16]|= 0x01; // Set JOG touch enable bit
  }

  else {
    masterTxBuffer[16] &= ~0x01; // Clear JOG touch enable bit
  }
/*
  if (slaveSPI.flag_transaction_completed)
    {
      for (int i=0; i<27; i++)
      {
          masterRxBuffer[i] = spi_slave_RXbuf[i];
          masterTxBuffer[i] = spi_slave_TXbuf[i];
      }
      slaveSPI.flag_transaction_completed = 0;
    }
*/
}


uint8_t calculateCRC(const uint8_t* data, size_t length) {
  uint16_t crc = 0;
  for (size_t i = 0; i < length; i++) {
    crc += data[i];
  }
  return crc%256;
}

void printFrame(const char* label, const uint8_t* data, size_t length) {
  Serial.print(label);
  for (size_t i = 0; i < length; i++) {
    Serial.printf(" 0x%02X", data[i]);
  }
  Serial.println();
}

void nonBlockingSPITransfer() {
  if (currentFrameIndex < sizeof(panelTxBuffer) / sizeof(panelTxBuffer[0])) {
    if (!isTransactionActive) {
      // Start a new frame transfer
      memcpy(currentFrame, panelTxBuffer[currentFrameIndex], 11);
      currentFrame[11] = calculateCRC(currentFrame, 11);
      currentByteIndex = 0;
      SPI2.beginTransaction(panelSPI);
      isTransactionActive = true;
      waitingForWaitPin = false; // Reset wait pin flag
    }

    if (isTransactionActive) {
      if (!waitingForWaitPin) {
        // Check if WAIT_PIN is HIGH
        if (digitalReadFast(CDJ_WAIT) == HIGH) {
          // Send the current byte and receive
          rxBuffer[currentByteIndex] = SPI2.transfer(currentFrame[currentByteIndex]);
          byteStartTime = micros();
          waitingForWaitPin = true; // Now wait for the byte delay

          currentByteIndex++;

          if (currentByteIndex >= FRAME_SIZE) {
            // Frame transfer complete
            SPI2.endTransaction();
            isTransactionActive = false;
            printFrame("Received Frame:", rxBuffer, FRAME_SIZE);
            frameStartTime = micros(); // Start the delay for the next frame
            currentFrameIndex++;
          }
        }
        // If WAIT_PIN is LOW, we do nothing and wait for it to go HIGH
      } else {
        // Waiting for the delay between bytes
        if (micros() - byteStartTime >= BYTE_DELAY_US) {
          waitingForWaitPin = false; // Byte delay finished, ready to send next byte
        }
      }
    } else {
      // Waiting for the delay between frames
      if (micros() - frameStartTime >= FRAME_DELAY_US) {
        // Ready to start the next frame
      }
    }
  } else {
    // All frames have been processed
    // You can set a flag or perform other actions here
    // to indicate the completion of the sequence.
    // For continuous operation, you might want to reset currentFrameIndex here.
    currentFrameIndex = 0;
  }
}


void createScreen(){
  screen = lv_obj_create(lv_scr_act());
  lv_obj_set_size(screen, 320, 320);
  lv_obj_set_style_bg_color(screen, lv_color_hex(0x000000),0);
  lv_obj_align(screen, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_border_width(screen, 0,0);
  lv_obj_remove_flag(screen, LV_OBJ_FLAG_SCROLLABLE);
}

void createVFD(){
  //lv_obj_t * vdf = lv_obj_create(screen);
  //lv_obj_set_size(vdf, 240, 240);
  //lv_obj_align(vdf,LV_ALIGN_CENTER, 0, 0);
  LV_IMAGE_DECLARE(Background_240px);
  LV_IMAGE_DECLARE(CDJ_CUE_MEMORY_240px);
  LV_IMAGE_DECLARE(CDJ_TOUCH_INDICATOR_240px);
  LV_IMAGE_DECLARE(CDJ_VINYL_INDICATOR_240px);
  LV_IMAGE_DECLARE(CDJ_POS_MARKER_120px);

  VDF_BG = lv_image_create(screen);
  lv_image_set_src(VDF_BG, &Background_240px);
  lv_obj_align(VDF_BG, LV_ALIGN_CENTER, 0, 0);


  VDF_CUE_MEM = lv_image_create(VDF_BG);
  lv_image_set_src(VDF_CUE_MEM, &CDJ_CUE_MEMORY_240px);
  lv_obj_align(VDF_CUE_MEM, LV_ALIGN_CENTER, 0, 0);


  VDF_TOUCH_IND = lv_image_create(VDF_BG);
  lv_image_set_src(VDF_TOUCH_IND, &CDJ_TOUCH_INDICATOR_240px);
  lv_obj_align(VDF_TOUCH_IND, LV_ALIGN_CENTER, 0, 0);


  VDF_VINYL_IND = lv_image_create(VDF_BG);
  lv_image_set_src(VDF_VINYL_IND, &CDJ_VINYL_INDICATOR_240px);
  lv_obj_align(VDF_VINYL_IND, LV_ALIGN_CENTER, 0, 0);



  vfd_scale = lv_scale_create(VDF_BG);
  lv_obj_remove_style_all(vfd_scale);
  lv_obj_set_size(vfd_scale, 240, 240);
  lv_scale_set_mode(vfd_scale, LV_SCALE_MODE_ROUND_INNER);
  lv_obj_set_style_bg_opa(vfd_scale, LV_OPA_COVER, 0);
  lv_obj_set_style_bg_color(vfd_scale, lv_palette_lighten(LV_PALETTE_RED, 5), 0);
  lv_obj_set_style_radius(vfd_scale, LV_RADIUS_CIRCLE, 0);
  lv_scale_set_label_show(vfd_scale, false);

  lv_obj_set_style_length(vfd_scale, 0, LV_PART_ITEMS);
  lv_obj_set_style_length(vfd_scale, 0, LV_PART_INDICATOR);
  lv_scale_set_range(vfd_scale, 0, 135);
  lv_scale_set_angle_range(vfd_scale, 360);
  lv_scale_set_rotation(vfd_scale, -90);
  lv_obj_set_style_bg_opa(vfd_scale, LV_OPA_0, 0);
 
  play_pos_obj = lv_image_create(vfd_scale);
  lv_image_set_src(play_pos_obj, &CDJ_POS_MARKER_120px);
  //For 320*320px object
  //lv_obj_set_pos(play_pos_obj, 160, 141); 
  //lv_image_set_pivot(play_pos_obj, 0, 19);
  lv_obj_set_pos(play_pos_obj, 120, 105);
  lv_image_set_pivot(play_pos_obj, -1, 15);
 
  lv_scale_set_image_needle_value(vfd_scale, play_pos_obj, 0);

}


void updateVDF(){
  static uint8_t lastVDFPos = 0;
  static bool lastTouchState = false;
  static bool lastVinylState = false;
  if (lastVDFPos != masterRxBuffer[4]) {
    lastVDFPos = masterRxBuffer[4];
    lv_scale_set_image_needle_value(vfd_scale, play_pos_obj, lastVDFPos);
  }

  if(lastTouchState != ((masterRxBuffer[3] & 0x01) !=0)) {
    lastTouchState = (masterRxBuffer[3] & 0x01) !=0;
    if (lastTouchState) {
      lv_obj_set_style_opa(VDF_TOUCH_IND, LV_OPA_COVER, 0);
    } else {
      lv_obj_set_style_opa(VDF_TOUCH_IND, LV_OPA_TRANSP, 0);
    }
  }

  if(lastVinylState != ((masterRxBuffer[3] & 0x02) !=0)) {
    lastVinylState = (masterRxBuffer[3] & 0x02) !=0;
    if (lastVinylState) {
      lv_obj_set_style_opa(VDF_VINYL_IND, LV_OPA_COVER, 0);
    } else {
      lv_obj_set_style_opa(VDF_VINYL_IND, LV_OPA_TRANSP, 0);
    }
  }
}



