#include "Adafruit_MCP23017.h"
#include <Wire.h>
#include <Arduino.h>
#ifndef MCP_H
#define MCP_H

/* 
* MCP_INT_MIRROR: if true, interrupts on either port A or port B cause both INTA and INTB to activate.
*  MCP_INT_ODR: If true, configures INTA and INTB as open drain; requires pullup resistor.
*               If false, INTA / INTB will not be floating; polarity will be set by INTPOL bit.
*  MCP_INT_POL: Polarity of INTA / INTB
*/
#define MCP_INT_MIRROR true  
#define MCP_INT_ODR  false 
#define MCP_INT_POL LOW

/*
* MCP23017 IO
* Datasheet: https://ww1.microchip.com/downloads/en/devicedoc/20001952c.pdf
* GPA0 through GPA7 (MCP pins 21-28) correspond to pins 0-7 using MCP library
* GPB0 through GPB7 (MCP pins 1-8) correspond to pins 8-15 using MCP library
*/
#define LED_3 4
#define LED_2 5
#define LED_1 8 // leftmost
#define PB_1 12
#define PB_2 10




class MCP23017 {
  
  static void isr0 ();
  static void isr1 ();

  public:
      // whichPin: either int0 (pin2) or int1 (pin3) on arduino
      static MCP23017 *mcp_instances [2];
      void start (const byte whichPin);
      void isr_0();
      void isr_1();
      void mcpWrite(int pin, int val);
      bool mcpRead(int pin);
      void mcpPinMode(int pin, int val);
      MCP23017* getMCPinstance(const byte whichPin) ;

  private:
};

#endif
