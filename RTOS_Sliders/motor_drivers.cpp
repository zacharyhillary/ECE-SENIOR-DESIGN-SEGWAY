#include "motor.h"
#include <Arduino.h>
class Motor{
    public:
        Motor(int pinNumber){//overloaded constructor
        
            pinMode(pinNumber, OUTPUT);// SET PIN TYPE
            pinNum=pinNumber; /
            setSpeed(0);// START WITH MOTORS NOT SPINNING!!!!
            }

        double speed;
        int pinNum

        void setSpeed(double input){
            if(input>100)speed=100; // BOUNDING THE INPUT SPEED
            else if(input<-100)speed=-100;// BOUNDING THE INPUT SPEED
            else speed=input;
            double output = (100 + speed)*1.275; // MAKING -100->100 input to a 0->255 output
            analogWrite(pinNum, output);

        }
};