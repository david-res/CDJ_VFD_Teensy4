#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>

DMAMEM uint16_t fb1[320*320] __attribute__ ((aligned(32)));
DMAMEM uint16_t fb2[320*320] __attribute__ ((aligned(32)));

#define ST7796_RST 10 //10
#define ST7796_DC 13 //13
#define ST7796_CS 11 //11
// WR pin 8
// RD pin 7
#define ST7796_BL 2


#define CDJ_WAIT 39
#define CDJ_RST 38

#define JOG_TOUCH 30
#define JOG_PULSE 25
#define JOG0 4
#define JOG1 5

#define ADCT A0
#define ADIN A1
#define LCD_BL 2


//SPI2 Pins on TMM for CDJ Panel
#define CDJ_MOSI_PIN 35
#define CDJ_MISO_PIN 34
#define CDJ_SCLK_PIN 37
#define CDJ_WAIT_PIN 38 // Pin to wait until HIGH
#define CDJ_RESET_PIN 39 // RST line


//SPI1 on TMM for SlaveSPI
#define slaveSPI SPI_SLAVE1
#define BYTE_BUF_SIZE 512

uint8_t slave_TXbuf[BYTE_BUF_SIZE];
uint8_t slave_RXbuf[BYTE_BUF_SIZE];



/*
These are commands sent from the Master control unit to the CDJ TMM panel.
These commands are copied from the panelTxBuffer and sent over SPI/I2C/Serial to the panel's panelTxBuffer.
[0 byte][bit0] = RED DIODE REVERSE
[0 byte][bit1] = PLAY LED
[0 byte][bit2] = CUE LED
[0 byte][bit3] = SD card red diode
[0 byte][bit4] = tempo reset green diode
[0 byte][bit5] = MT red diode on button
[0 byte][bit7] = CDJ green diode
[1 byte][bit0] = RED DIODE HOT CUE A
[1 byte][bit1] = GREEN DIODE HOT CUE A
[1 byte][bit2] = RED DIODE HOT CUE B
[1 byte][bit3] = GREEN DIODE HOT CUE B
[1 byte][bit4] = RED DIODE HOT CUE C
[1 byte][bit5] = GREEN DIODE HOT CUE C
[1 byte][bit6] = LOOP IN LED
[1 byte][bit7] = LOOP OUT LED
[2 byte][bit0] = Vinyl blue diode
[3 byte][bit 0] = JOG touch indicator
[3 byte][bit 1] = JOG Vinyl indiocator
[4 byte] = JOG indicator position (0…135)
*/
uint8_t masterRxBuffer[18]; 


/*
[frame 9 ][9 byte][bit0] = RED DIODE REVERSE
[frame 9 ][9 byte][bit1] = PLAY LED
[frame 9 ][9 byte][bit2] = CUE LED
[frame 9 ][9 byte][bit3] = SD card red diode
[frame 9 ][9 byte][bit4] = tempo reset green diode
[frame 9 ][9 byte][bit5] = MT red diode on button
[frame 9 ][9 byte][bit7] = CDJ green diode


[frame 10][1 byte][bit0] = RED DIODE HOT CUE A
[frame 10][1 byte][bit1] = GREEN DIODE HOT CUE A
[frame 10][1 byte][bit2] = RED DIODE HOT CUE B
[frame 10][1 byte][bit3] = GREEN DIODE HOT CUE B
[frame 10][1 byte][bit4] = RED DIODE HOT CUE C
[frame 10][1 byte][bit5] = GREEN DIODE HOT CUE C
[frame 10][1 byte][bit6] = LOOP IN LED
[frame 10][1 byte][bit7] = LOOP OUT LED

[frame 10][2 byte][bit0] = Vinyl blue diode


*/
uint8_t panelTxBuffer[2][11]={
  {0x9, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00}, 
  {0xA, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
};


/*
[byte 2][bit 4] = Track Search BWD
[byte 3][bit 6] = CUE
[byte 4][bit 7] = CUE A
[byte 4][bit 6] = CUE B
[byte 4][bit 5] = CUE C
[byte 4][bit 4] = CUE Rec
[byte 4][bit 3] = Time mode
[byte 5][bit 7] = Loop IN
[byte 5][bit 6] = Loop OUT
[byte 5][bit 5] = Loop Reloop/Exit
[byte 5][bit 4] = Wave Search BWD
[byte 5][bit 3] = Call Search BWD
[byte 5][bit 2] = Memory
[byte 5][bit 1] = Delete
[byte 6][bit 7] = Tempo Range
[byte 6][bit 6] = Master Tempo
[byte 6][bit 5] = Tempo Reset
[byte 6][bit 4] = Eject
[byte 6][bit 2] = Jog Mode
[byte 6][bit 1] = Wave Search FWD
[byte 6][bit 0] = Call Search FWD
[byte 7][bit 1] = Lock
[byte 7][bit 0] = Play Direction
[byte 8][--] = TOUCH/BREAK (0-254)
[byte 9][--] = RELEASE/START (0-254)
[byte 10][--] = PITCH MSB (0-1023)
[byte 11][--] = PITCH LSB (0-1023)
[byte 12][--] = PITCH CENTER MSB (0-1023)
[byte 13][--] = PITCH CENTER LSB (0-1023)
[byte 14][--] = JOG POSITION MSB (0-65535)
[byte 15][--] = JOG POSITION LSB (0-65535)
[byte 16][bit 0] = JOG direction (1 = forward, 0 = reverse)
[byte 16][bit 1] = JOG touch enable (1 = enabled, 0 = disabled)
*/
volatile uint8_t masterTxBuffer[18];



/*

These are commands sent from the CDJ panel to the Slave control unit (this Teensy).
[byte 2][bit 4] = Track Search BWD
[byte 3][bit 6] = CUE
[byte 4][bit 7] = CUE A
[byte 4][bit 6] = CUE B
[byte 4][bit 5] = CUE C
[byte 4][bit 4] = CUE Rec
[byte 4][bit 3] = Time mode
[byte 5][bit 7] = Loop IN
[byte 5][bit 6] = Loop OUT
[byte 5][bit 5] = Loop Reloop/Exit
[byte 5][bit 4] = Wave Search BWD
[byte 5][bit 3] = Call Search BWD
[byte 5][bit 2] = Memory
[byte 5][bit 1] = Delete
[byte 6][bit 7] = Tempo Range
[byte 6][bit 6] = Master Tempo
[byte 6][bit 5] = Tempo Reset
[byte 6][bit 4] = Eject
[byte 6][bit 2] = Jog Mode
[byte 6][bit 1] = Wave Search FWD
[byte 6][bit 0] = Call Search FWD
[byte 7][bit 1] = Lock
[byte 7][bit 0] = Play Direction
[byte 8][--] = TOUCH/BREAK (0-254)
[byte 9][--] = RELEASE/START (0-254)
[byte 10][--] = CRC
*/
uint8_t panelRxBuffer[12] = {0x0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //CRC is last byte and not defined here

#endif // GLOBALS_H