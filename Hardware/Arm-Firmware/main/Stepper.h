#include <Arduino.h>
#include "MCP23017.h"
#ifndef STEPPER_H
#define STEPPER_H

class Stepper {
    public:
        Stepper(int, int, int, int);
        void enableMotor();
        void disableMotor();
        void setSpeed(int);
        void moveToAngle(int);
        int getCurrentStep();
        void test();
        void connectMCP();
        void stepCW(unsigned long int);
        void stepCCW(unsigned long int);
    private:
        int enable;
        int pulse;
        int dir;
        int stepsperrev;
        unsigned long stepdelay;
        //static MCP23017 *mcpArray;
        int currentStep;
        // void stepCW(int);
        // void stepCCW(int);
        void step(unsigned long int);
};
#endif
