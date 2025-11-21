#include "T4_DMA_SPI_SLAVE.h"

volatile int n = 0;

#define BYTE_BUF_SIZE 512

#define MOSI_SNIFFER SPI_SLAVE


uint8_t TXbuf[BYTE_BUF_SIZE];
uint8_t RXbuf[BYTE_BUF_SIZE];

int Nbytes = 27;


void setup()
{
  Serial.begin(115200);
  while (!Serial);

//  Serial.print(CrashReport);
  Serial.println("\n" __FILE__ " " __DATE__ " " __TIME__);
  Serial.println("T4_DMA_SPI_SLAVE Example - 4 wire SPI bus MODE0 Slave ");
 
  for (int i=0; i<BYTE_BUF_SIZE;i++){ //Set TXbuf with some static values
    TXbuf[i] = i;
  }

  MOSI_SNIFFER.begin(SPI_MODE0, MSBFIRST, MODE_4WIRE);
  MOSI_SNIFFER.auto_repeat = 1;
  MOSI_SNIFFER.print_pin_use();
  //MOSI_SNIFFER.debug();

  prepare_for_next_transaction();

  Serial.printf("begin done\n");
}

void prepare_for_next_transaction()
{
  MOSI_SNIFFER.prepare_for_slave_transfer(TXbuf, RXbuf, Nbytes);
}

void loop()
{
  if (MOSI_SNIFFER.flag_transaction_completed)
  {
//    Serial.printf ("n=%d, ", n++);
    
    Serial.printf ("\nRECEIVED ");
    for (int i=0; i<Nbytes; i++)
    {
        Serial.printf ("%02x ", RXbuf[i]);
    }
    Serial.printf ("\nSENT     ");
    for (int i=0; i<Nbytes; i++)
    {
        Serial.printf ("%02x ", TXbuf[i]);
    }
    Serial.printf ("\n");
    
//    delay(1000);
    
    MOSI_SNIFFER.flag_transaction_completed = 0;
  }

}
