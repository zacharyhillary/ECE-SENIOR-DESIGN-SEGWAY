#ifndef MOTOR_H
#define MOTOR_H

class Motor {
public:
    Motor(int pinNumber);
    void setSpeed(double input);

private:
    int pinNum;
    double speed;
};

#endif // MOTOR_H