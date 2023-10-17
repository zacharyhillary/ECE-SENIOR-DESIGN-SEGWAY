#include "motor_drivers.h"
#include "mpu_6050_drivers.h"
#include <Arduino.h>

Motor::Motor(int pinNumber) {  //overloaded constructor
  pinMode(pinNumber, OUTPUT);  // SET PIN TYPE
  pinNum = pinNumber;
  setSpeed(0);  // START WITH MOTORS NOT SPINNING!!!!
}

void Motor::setSpeed(double input) {
    if (input > 100) speed = 100;         // BOUNDING THE INPUT SPEED
    else if (input < -100) speed = -100;  // BOUNDING THE INPUT SPEED
    else speed = input;
    double output = (100 + speed) * 1.275;  // MAKING -100->100 input to a 0->255 output
    analogWrite(pinNum, output);
}
