#include "motor.h"
#include <Arduino.h>
class Motor{
    public:
        Motor(int pinNumber){//overloaded constructor
            pinNum=pinNumber;
            }
        double speed;
        int pinNum
        void setSpeed(double input){
            if(input>100)speed=100;
           else if(input<-100)speed=-100;
           else speed=input;
            double output = (100 + speed)*1.275;
            analogWrite(pinNum, output);

        }
};