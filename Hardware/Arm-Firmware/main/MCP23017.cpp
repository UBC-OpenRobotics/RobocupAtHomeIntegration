#include "MCP23017.h"

Adafruit_MCP23017 mcp0;
Adafruit_MCP23017 mcp1;

void MCP23017::start (const byte whichPin) {
   switch (whichPin) {
   case 2:
      mcp_instances[0] = this;
      mcp0.begin(); //Adafruit library
      mcp0.pinMode(LED_1, OUTPUT);
      mcp0.pinMode(PB_1,INPUT); 
      mcp0.pullUp(PB_1,HIGH); 
      mcp0.setupInterrupts(MCP_INT_MIRROR, MCP_INT_ODR, MCP_INT_POL);
      mcp0.setupInterruptPin(PB_1,CHANGE);
      mcp0.readGPIOAB(); 
      attachInterrupt(digitalPinToInterrupt(2), isr0, FALLING);

      for (int i = 0; i < 16; i++) {
         mcp0.pinMode(i, OUTPUT);
         mcp0.digitalWrite(i, HIGH);
      }
      break;
   case 3:
      mcp_instances[1] = this;
      mcp1.begin(1); //Adafruit library
      mcp1.pinMode(LED_2, OUTPUT);
      mcp1.pinMode(LED_3, OUTPUT);
      mcp1.pinMode(PB_2,INPUT); 
      mcp1.pullUp(PB_2,HIGH); 
      mcp1.setupInterrupts(MCP_INT_MIRROR, MCP_INT_ODR, MCP_INT_POL);
      mcp1.setupInterruptPin(PB_2,CHANGE);
      mcp1.readGPIOAB(); 
      attachInterrupt(digitalPinToInterrupt(3), isr1, FALLING);
      break;
   } // end switch
} // end MCP23017::start

static void MCP23017::isr0 () {
    if (MCP23017::mcp_instances[0] != NULL) {
      MCP23017::mcp_instances[0] -> isr_0 ();
      attachInterrupt(digitalPinToInterrupt(2),isr0,FALLING); 
    }
}
static void MCP23017::isr1 () {
   if (MCP23017::mcp_instances[1] != NULL) {
      MCP23017::mcp_instances[1] -> isr_1 ();  
      attachInterrupt(digitalPinToInterrupt(3),isr1,FALLING); 
   }
}

void MCP23017::isr_0() {
   uint8_t pin,val;
   static uint16_t ledState = 0;

   noInterrupts();
   delayMicroseconds(1000); //debounce
   detachInterrupt(digitalPinToInterrupt(2));
   interrupts(); 

   pin = mcp0.getLastInterruptPin();
   val = mcp0.getLastInterruptPinValue();
   Serial.print("value:");
   Serial.println(val);
   Serial.print("pin: ");
   Serial.println(pin);
   Serial.print("LED: ");
   Serial.println(ledState);
   
   if ( pin == PB_1 && val == 1) { 
      if (ledState) { mcp0.digitalWrite(LED_1, LOW); }
      else { mcp0.digitalWrite(LED_1, HIGH); }
      ledState = ! ledState;
   }
   attachInterrupt(digitalPinToInterrupt(2),isr0,FALLING); 
}

void MCP23017::isr_1() {
   uint8_t pin,val;
   static uint16_t ledState = 0;

   noInterrupts();
   delayMicroseconds(1000); //debounce
   detachInterrupt(digitalPinToInterrupt(3));
   interrupts(); 

   pin = mcp1.getLastInterruptPin();
   val = mcp1.getLastInterruptPinValue();
   Serial.print("value:");
   Serial.println(val);
   Serial.print("pin: ");
   Serial.println(pin);
   Serial.print("LED: ");
   Serial.println(ledState);
   
   if ( pin == PB_2 && val == 1) { 
      if (ledState) { mcp1.digitalWrite(LED_2, LOW); }
      else { mcp1.digitalWrite(LED_2, HIGH); }
      ledState = ! ledState;
   }
   attachInterrupt(digitalPinToInterrupt(3),isr1,FALLING); 
}



void MCP23017::mcpPinMode(int pin, int val) {
   if (mcp_instances [0] == this) {
      switch (val) {   
         case INPUT: 
            mcp0.pinMode(pin, INPUT);
            break;
         case INPUT_PULLUP:
            mcp0.pinMode(pin, INPUT);
            mcp0.pullUp(pin, HIGH);
            break;
         case OUTPUT:
            mcp0.pinMode(pin, OUTPUT);
            break;
      } //end switch
   }
   else if (mcp_instances [1] == this) {
      switch (val) {   
         case INPUT: 
            mcp1.pinMode(pin, INPUT);
            break;
         case INPUT_PULLUP:
            mcp1.pinMode(pin, INPUT);
            mcp1.pullUp(pin, HIGH);
            break;
         case OUTPUT:
            mcp1.pinMode(pin, OUTPUT);
            //Serial.println("set to output: ");
            //Serial.println(pin);
            break;
      } //end switch
   }
}

void MCP23017::mcpWrite(int pin, int val) {
   if (mcp_instances [0] == this) { mcp0.digitalWrite(pin, val); }
   else if (mcp_instances [1] == this) { mcp1.digitalWrite(pin, val); }
}

bool MCP23017::mcpRead(int pin) {
   if (mcp_instances [0] == this) { return mcp0.digitalRead(pin); }
   else if (mcp_instances [1] == this) { return mcp1.digitalRead(pin); }
}



MCP23017* MCP23017::getMCPinstance(const byte whichPin) {
   switch (whichPin) {
      case 2:
         return mcp_instances[0];
      case 3:
         return mcp_instances[1];
   } // end switch
   
}
