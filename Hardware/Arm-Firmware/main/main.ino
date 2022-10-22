#include <Arduino.h>
#include "MCP23017.h"
#include "Stepper.h"
#include <Adafruit_MCP23017.h>
#define PULpin 10
#define DIRpin 11
#define ENApin 9
int STEPSPERREV = 4000;
long unsigned int stepthismuch = STEPSPERREV*50;
unsigned long stepdelay = 0;
MCP23017 mcp_pin2; // the chip corresponding to this class interrupts on arduino pin 2
MCP23017 mcp_pin3; // the chip corresponding to this class interrupts on arduino pin 3
MCP23017* MCP23017::mcp_instances [2] = { NULL, NULL };
Stepper myStepper(ENApin, PULpin, DIRpin, STEPSPERREV);
int steps = 0;
void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode (2, INPUT_PULLUP); // arduino interrupt pin
  pinMode (3, INPUT_PULLUP); // arduino interrupt pin
  mcp_pin2.start(2); // start instance of MCP23017
  mcp_pin3.start(3); // start instance of MCP23017
  myStepper.connectMCP();
  myStepper.enableMotor();
  myStepper.setSpeed(200);
}

void loop() {
  myStepper.stepCW(200000);
  //myStepper.test();
  
  //step(200000);
  delay(100);
}

void step(unsigned long int steps) {
    unsigned long lastStepTime = 0;
    unsigned long int pulses = steps * 2;
    while(pulses > 0) {
        unsigned long time = micros();
        if (time - lastStepTime >= 2) {
            lastStepTime = time;
            pulses--;
            if(pulses % 2 == 0) {
                digitalWrite(7, HIGH);
            } else {
                digitalWrite(7, LOW);
            }
        }
    }
    Serial.println("Finished stepping.");
}
