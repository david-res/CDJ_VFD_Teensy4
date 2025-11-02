#ifndef _ST7796s_t4_mm_H_
#define _ST7796s_t4_mm_H_

const int WR_PIN[] = {8, 16};
const int RD_PIN[] = {7, 17};

#include "Arduino.h"
#include "FlexIO_t4.h"
#include "DMAChannel.h"

#define ST7796S

#define SHIFTNUM 4 // number of shifters used (must be 1, 2, 4, or 8)
#define SHIFTER_DMA_REQUEST 3 // only 0, 1, 2, 3 expected to work


#define DATABUFBYTES (320*320)/4


#define _TFTWIDTH   320      // ST7796S TFT width in default rotation
#define _TFTHEIGHT  320      // ST7796S TFT height in default rotation

#define ST7796S_NOP         0x00  // No-op
#define ST7796S_SWRESET     0x01  // Software reset
#define ST7796S_RDDID       0x04  // Read display ID
#define ST7796S_RDDST       0x09  // Read display status

#define ST7796S_SLPIN       0x10  // Enter Sleep Mode
#define ST7796S_SLPOUT      0x11  // Sleep Out
#define ST7796S_PTLON       0x12  // Partial Mode ON
#define ST7796S_NORON       0x13  // Normal Display Mode ON

#define ST7796S_RDMODE      0x0A  // Read Display Power Mode
#define ST7796S_RDMADCTL    0x0B  // Read Display MADCTL
#define ST7796S_RDCOLMOD    0x0C  // Read Display Pixel Format
#define ST7796S_RDIMGFMT    0x0D  // Read Display Image Mode
#define ST7796S_RDDSM       0x0E  // Read Display Signal Mode
#define ST7796S_RDSELFDIAG  0x0F  // Read Display Self-Diagnostic Result

#define ST7796S_INVOFF      0x20  // Display Inversion OFF
#define ST7796S_INVON       0x21  // Display Inversion ON
#define ST7796S_GAMMASET    0x26  // Gamma Set
#define ST7796S_DISPOFF     0x28  // Display OFF
#define ST7796S_DISPON      0x29  // Display ON

#define ST7796S_CASET       0x2A  // Column Address Set 
#define ST7796S_PASET       0x2B  // Page Address Set 
#define ST7796S_RAMWR       0x2C  // Memory Write 
#define ST7796S_RAMRD       0x2E  // Memory Read

#define ST7796S_PTLAR       0x30  // Partial Area
#define ST7796S_TEOFF       0x34  // Tearing effect line off
#define ST7796S_TEON        0x35  // Tearing effect line on
#define ST7796S_MADCTL      0x36  // Memory Access Control
#define ST7796S_VSCRSADD    0x37  // Vertical Scrolling Start Address
#define ST7796S_COLMOD      0x3A  // Interface pixel format

#define ST7796S_TESLWR      0x44  // Write tear scan line

#define ST7796S_FRMCTR1     0xB1  // Frame Rate Control (Normal Mode / Full Colors)
#define ST7796S_FRMCTR2     0xB2  // Frame Rate Control (Idle Mode / 8 Colors)
#define ST7796S_FRMCTR3     0xB3  // Frame Rate Control (Partial Mode / Full Colors)
#define ST7796S_INVCTR      0xB4  // Display Inversion Control
#define ST7796S_DFUNCTR     0xB6  // Display Function Control
#define ST7796S_ETMOD       0xB7  // Entry Mode Set

#define ST7796S_PWCTR1      0xC0  // Power Control 1
#define ST7796S_PWCTR2      0xC1  // Power Control 2
#define ST7796S_PWCTR3      0xC2  // Power Control 3 (For Normal Mode)
#define ST7796S_PWCTR4      0xC3  // Power Control 4 (For Idle Mode)
#define ST7796S_PWCTR5      0xC4  // Power Control 5 (For Partial Mode)
#define ST7796S_VMCTR1      0xC5  // VCOM Control
#define ST7796S_CABCCTRL1   0xC6  // CABC Control 1
#define ST7796S_CABCCTRL2   0xC8  // CABC Control 2

#define ST7796S_PGAMCTRL    0xE0  // Positive Gamma Control
#define ST7796S_NGAMCTRL    0xE1  // Negative Gamma Control
#define ST7796S_SETIMAGE    0xE9  // Set Image Function

#define ST7796S_RDID1       0xDA  // Read ID1 value
#define ST7796S_RDID2       0xDB  // Read ID2 value
#define ST7796S_RDID3       0xDC  // Read ID3 value

#define MADCTL_MY  0x80  // Bottom to top
#define MADCTL_MX  0x40  // Right to left
#define MADCTL_MV  0x20  // Row/Column exchange
#define MADCTL_ML  0x10  // LCD refresh Bottom to top
#define MADCTL_RGB 0x00  // Red-Green-Blue pixel order
#define MADCTL_BGR 0x08  // Blue-Green-Red pixel order
#define MADCTL_MH  0x04  // LCD refresh right to left
#define MADCTL_GS  0x01
#define MADCTL_SS  0x02

//MADCTL 0,1,2,3 for setting rotation and 4 for screenshot
#if defined (ST7796S) || defined (ILI9486)
#define MADCTL_ARRAY { MADCTL_MX | MADCTL_BGR, MADCTL_MV | MADCTL_BGR, MADCTL_MY | MADCTL_BGR, MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR, MADCTL_MY | MADCTL_MV | MADCTL_BGR } // ST7796S/9486
#elif defined (ILI9481_1) || defined (ILI9481_2)
#define MADCTL_ARRAY { MADCTL_BGR | MADCTL_SS, MADCTL_MV | MADCTL_BGR, MADCTL_BGR | MADCTL_GS, MADCTL_MV | MADCTL_BGR | MADCTL_SS | MADCTL_GS } // ILI9481
#elif defined (R61529)
#define MADCTL_ARRAY { MADCTL_RGB, MADCTL_MV | MADCTL_MX | MADCTL_RGB, MADCTL_RGB | MADCTL_GS | MADCTL_MX, MADCTL_MV | MADCTL_RGB | MADCTL_GS } // R61529
#endif

#ifdef __cplusplus
class ST7796s_t4_mm {
  public:
    ST7796s_t4_mm(int8_t dc, int8_t cs = -1, int8_t rst = -1);
    void begin(uint8_t buad_div = 20);
    void setRotation(uint8_t r);
    void invertDisplay(bool invert);
    void setAddrWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
    void pushPixels16bit(const uint16_t * pcolors, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
    void pushPixels16bitDMA(const uint16_t * pcolors, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

    void DMAerror();

    typedef void(*CBF)();
    CBF _callback;
    void onCompleteCB(CBF callback);
    
  private:

  FlexIOHandler *pFlex;
  IMXRT_FLEXIO_t *p;
  const FlexIOHandler::FLEXIO_Hardware_t *hw;
  static DMAChannel flexDma;
   
    uint8_t _buad_div = 20; 

    uint8_t _bitDepth = 16;
    uint8_t _rotation = 0;
    uint8_t MADCTL[5];

    uint8_t _frameRate = 60;

    bool _bTearingOn = false;
    uint16_t _tearingScanLine = 0;

    int16_t _width, _height;
    int8_t  _dc, _cs, _rst;

    uint8_t _dummy;
    uint8_t _curMADCTL;

    uint16_t _lastx1, _lastx2, _lasty1, _lasty2;

    volatile bool WR_DMATransferDone = true;
    uint32_t MulBeatCountRemain;
    uint16_t *MulBeatDataRemain;
    uint32_t TotalSize; 

    void displayInit();
    void CSLow();
    void CSHigh();
    void DCLow();
    void DCHigh();
    void gpioWrite();
    void gpioRead();
    
    void FlexIO_Init();
    void FlexIO_Config_SnglBeat();
    void FlexIO_Clear_Config_SnglBeat();
    void FlexIO_Config_MultiBeat();

    void SglBeatWR_nPrm_8(uint32_t const cmd, uint8_t const *value , uint32_t const length);
    void SglBeatWR_nPrm_16(uint32_t const cmd, const uint16_t *value, uint32_t const length);
    void MulBeatWR_nPrm_DMA(uint32_t const cmd,  const void *value, uint32_t const length);

    void microSecondDelay();

    static void dmaISR();
    void flexDma_Callback();

    bool isCB = false;
    void _onCompleteCB();
    
    static ST7796s_t4_mm *dmaCallback;
    
};
#endif //__cplusplus
#endif //_IST7796s_t4_mm_H_