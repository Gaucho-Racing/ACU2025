#include "MyFlags.h"
#include <MCP3x0x.h>
#include <SPI.h>
//#include <avr/sleep.h>


#define clkWake  0b00000000 // 20MHz
#define clkSleep 0b00010001 // 3.33MHz


MCP3008 ADCs[4];
const int CS[4] = {PIN_PA7, PIN_PA6, PIN_PA5, PIN_PA4};


volatile uint8_t Volts[33];
volatile uint8_t Cursor;
volatile uint8_t TxByte = 0;
volatile bool measure = false;

//void I2C_RxHandler(int numBytes)
//{
//  TWI0.SCTRLB = TWI_ACKACT_ACK_gc;
////  while(Wire.available()) {  // Read Any Received Data
////    Cursor = Wire.read();
////  }
//  Cursor = TWI0.SDATA;
//}

//void I2C_TxHandler(void)
//{
//  Wire.write(Volts[Cursor]);
//  TWI0.SCTRLB = TWI_ACKACT_NACK_gc;
//  Cursor++;
//}


ISR(TWI0_TWIS_vect) {
  CCP = 0xD8;
  CLKCTRL.MCLKCTRLB = clkWake;
  if (TWI0.SSTATUS & 0b00000010) { // host read
    TWI0.SDATA = Volts[Cursor];
    TWI0.SCTRLB = 0b00000011;
  }
  else if (TWI0.SSTATUS & 0b01000000) { // addr, host write
    TWI0.SCTRLB = 0b00000011;
  }
  else { // after host write data
    Cursor = TWI0.SDATA;
    TWI0.SCTRLB = 0b00000010;
  }
}

ISR(TCB0_INT_vect) {
  //  CCP = 0xD8;
  //  CLKCTRL.MCLKCTRLB = clkWake;
  TCB0.INTFLAGS = 0b11;
  measure = true;
}


void setup() {
  // put your setup code here, to run once:
  for (uint8_t cid = 0; cid < 4; cid++) {
    ADCs[cid].setCsPin(CS[cid]);
    ADCs[cid].setVref(5);
    ADCs[cid].setFsck(2500000);
    ADCs[cid].begin();
  }

  //  Wire.begin(0b1010000);
  //  Wire.onReceive(I2C_RxHandler);
  //  Wire.onRequest(I2C_TxHandler);
  //TWI0.CTRLA  = 0b00011100;
  TWI0.SADDR  = 0b10100000;
  TWI0.SCTRLA = 0b11000001;

  pinMode(PIN_PB2, OUTPUT);
  pinMode(PIN_PB3, OUTPUT);

  AC0.CTRLA = 0;
  ADC0.CTRLA = 0;
  USART0.CTRLB = 0;
  RTC.CTRLA = 0;
  TCA0.SPLIT.CTRLA = 0;
  CCL.CTRLA = 0;

  TCB0.CTRLA   = 0b01000011; // for wakeup interrupts
  TCB0.CTRLB   = 0b00000000;
  TCB0.INTCTRL = 0b00000001;
  TCB0.DBGCTRL = 0b00000001;
  TCB0.CCMP    = 65535;

  //SLPCTRL.CTRLA = 0b011;
  sei();

  while (1) {
    if (measure) {
      CCP = 0xD8;
      CLKCTRL.MCLKCTRLB = clkWake;
      for (uint8_t cid = 0; cid < 4; cid++) {
        for (uint8_t ch = 0; ch < 8; ch++) {
          Volts[(cid << 3) + ch + 1] = ADCs[cid].readRaw(1, ch) >> 2;
        }
      }
      measure = false;
      CCP = 0xD8;
      CLKCTRL.MCLKCTRLB = clkSleep;
    }
    //sleep_cpu();
  }
}

void loop() {
  // put your main code here, to run repeatedly:
}
