#include "Arduino_FreeRTOS.h"
#include "mpu_6050_drivers.h"
#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>

double currentAngle;
double steering;



SoftwareSerial SWSerial(NOT_A_PIN, 16);  // RX on no pin (unused), TX on pin 11 (to S1).
SabertoothSimplified ST(SWSerial);       // We'll name the Sabertooth object ST.


void GetAngleTask(void* pvParameters) {
  int motorPower;
  while (1) {
    currentAngle = -1 * (Get_Angle() + 7);  // set global variable to the current angle of the segway
    vTaskDelay(pdMS_TO_TICKS(16));
  }
}




bool riderMode = false;
double previousError = 0;
double integral = 0;

double ComputePID(double kp, double ki, double kd, double setpoint, double processVariable) {
  double error = setpoint - processVariable;
  integral += ki * error;
  double derivative = error - previousError;
  double PIDoutput = kp * error + integral + kd * derivative;
  previousError = error;
  return PIDoutput;
}

double output;
void MainControlTask(void* pvParameters) {

  bool previousRiderMode = !riderMode;  // SET IT so it goes into if statement on first loop
  vTaskDelay(pdMS_TO_TICKS(500));       // LET IMU STABILISE
  long timerMillis;

  const double slowKp = 4;
  const double slowKd = 0;

  const double riderKp = 9;
  const double riderKi = 0;
  const double riderKd = 12.5;
  const double riderSetpoint = 0;

  const double noRiderKp = 9;
  const double noRiderKi = 0.15;
  const double noRiderKd = 15;
  const double noRiderSetpoint = 7.4;

  const double LEFT_MOTOR_SCALE = 1.10; // 10 percent increase

  double kp;
  double ki;
  double kd;
  bool resetConfig;
  double setpoint;

  while (1) {
    if (previousRiderMode != riderMode) {  //gets activated when you switch rider mode AND FIRST TIME BOOT UP
      previousRiderMode = riderMode;
      integral = 0;
      resetConfig = true;
      timerMillis = millis();
      kp = slowKp;
      kd = slowKd;
      ki = 0;
      if (riderMode) setpoint = riderSetpoint;  //if a rider is present
      else setpoint = noRiderSetpoint;          //if rider is not present
    }


    if (millis() - timerMillis >= 1000 && resetConfig) {  //put kp ki kd back to normal
      if (riderMode) {                                    //if a rider is present
        kp = riderKp;
        ki = riderKi;
        kd = riderKd;
        setpoint = riderSetpoint;
      } else {  //if rider is not present
        kp = noRiderKp;
        ki = noRiderKi;
        kd = noRiderKd;
        setpoint = noRiderSetpoint;
      }
      resetConfig = false;
    }

    output = ComputePID(kp, ki, kd, setpoint, currentAngle);

    if (output > 127) {  //bound the output
      output = 127;
    }
    if (output < -127) {  //bound the output
      output = -127;
    }

    if (currentAngle > 45 || currentAngle < -45) {  // SAFETY CODE TO SHUT DOWN SEGWAY in case extreme angle
      output = 0;
      ST.motor(1, output);
      ST.motor(2, output);
      vTaskEndScheduler();
    }


    int leftMotorOutput;
    int rightMotorOutput;

    if (steering >= 1) {  //right turn
      rightMotorOutput = output - steering;
      leftMotorOutput = output + steering;
    } else if (steering <= -1) {  //left turn
      leftMotorOutput = output + steering;
      rightMotorOutput = output - steering;
    } else {  //no turn
      leftMotorOutput = output;
      rightMotorOutput = output;
    }

    // if (leftMotorOutput > 0 && leftMotorOutput <= 6) leftMotorOutput = 6;  //motor deadzone
    // else if (leftMotorOutput <= 0 && leftMotorOutput >= -6) leftMotorOutput = -6;
    // if (rightMotorOutput > 0 && rightMotorOutput <= 5) rightMotorOutput = 5;
    // else if (rightMotorOutput <= 0 && rightMotorOutput >= -5) rightMotorOutput = -5;

    ST.motor(1, leftMotorOutput * LEFT_MOTOR_SCALE);   // left motor
    ST.motor(2, rightMotorOutput);  // right motor


    vTaskDelay(pdMS_TO_TICKS(10));  // ~60 Hz need to implement with different timing source if we want faster timing
  }
}


enum SteeringDirection {
  STEERING_LEFT,
  STEERING_RIGHT,
  NONE
};

void pushNextInput(int* lastThreeInputs, int nextInput) {
  lastThreeInputs[0] = lastThreeInputs[1];
  lastThreeInputs[1] = lastThreeInputs[2];
  lastThreeInputs[2] = nextInput;
}

void SteeringTask(void* pvParameters) {
  int leftSteeringPin = A2;
  int rightSteeringPin = A3;
  int targetSteering = 0;
  int currentSteering = 0;
  int lastThreeInputs[] = {0, 0, 0};
  pinMode(leftSteeringPin, INPUT);
  pinMode(rightSteeringPin, INPUT);
  while (1) {
    double rightSteeringRaw = analogRead(rightSteeringPin);
    double leftSteeringRaw = analogRead(leftSteeringPin);
    if (rightSteeringRaw < 700 && leftSteeringRaw > 700) {
      targetSteering = 100;
      pushNextInput(lastThreeInputs, 100);
    } else if (rightSteeringRaw > 700 && leftSteeringRaw < 700) {
      targetSteering = -100;
      pushNextInput(lastThreeInputs, -100);
    } else {
      targetSteering = 0;
      pushNextInput(lastThreeInputs, 0);
    }

    if (lastThreeInputs[0] == 0 && lastThreeInputs[1] == 0 && lastThreeInputs[2] == 0) {
      currentSteering = 0;
    } else if (currentSteering < targetSteering) {
      currentSteering = min(100, currentSteering + 15);
    } else if (currentSteering > targetSteering) {
      currentSteering = max(-100, currentSteering - 15);
    }

    // double rightSteering = -1 * (((double)analogRead(rightSteeringPin) - 40.0) * (1.0 / 9.0) - 100);
    // double leftSteering = -1 * (((double)analogRead(leftSteeringPin) - 14.0) * (1.0 / 9.0) - 100);
    // steering = rightSteering - leftSteering;                  // ALL LEFT = -100 ALL RIGHT = 100 NONE = 0



    steering = (0.3 - output * 0.0022) * currentSteering;            // 0.3*steering at 0 output and and .1*steering at 90
    //if ((steering <= 20) && (steering >= -20)) steering = 0;  // steering deadzone;
    vTaskDelay(pdMS_TO_TICKS(75));
  }
}


#define TURN_SIGNAL_OUTPUT_1 26
#define TURN_SIGNAL_OUTPUT_2 27
#define TURN_SIGNAL_INPUT_1 28
#define TURN_SIGNAL_INPUT_2 29
bool signalLevelOne = true;
bool signalLevelTwo = true;
void turnSignalTask(void* pvParameters) {
  pinMode(TURN_SIGNAL_OUTPUT_1, OUTPUT);
  pinMode(TURN_SIGNAL_OUTPUT_2, OUTPUT);
  pinMode(TURN_SIGNAL_INPUT_1, INPUT_PULLUP);
  pinMode(TURN_SIGNAL_INPUT_2, INPUT_PULLUP);
  while (1) {
    int pin1 = digitalRead(TURN_SIGNAL_INPUT_1);
    int pin2 = digitalRead(TURN_SIGNAL_INPUT_2);
    if (pin1 == LOW) {
      //digitalWrite(TURN_SIGNAL_OUTPUT_1, signalLevelOne);
      //signalLevelOne = !signalLevelOne;//toggle
      riderMode = true;
    } else {
      digitalWrite(TURN_SIGNAL_OUTPUT_1, HIGH);
      riderMode = false;
    }
    if (pin2 == LOW) {
      //digitalWrite(TURN_SIGNAL_OUTPUT_2, signalLevelTwo);
      //signalLevelTwo = !signalLevelTwo;
    } else {
      digitalWrite(TURN_SIGNAL_OUTPUT_2, HIGH);
    }
    vTaskDelay(pdMS_TO_TICKS(250));  // Delay for 0.25 seconds
  }
}

void setup() {
  SWSerial.begin(9600);
  ST.motor(1, 0);
  ST.motor(2, 0);
  mpu_setup();
  Serial1.begin(9600);
  xTaskCreate(GetAngleTask, "GetAngleTask", configMINIMAL_STACK_SIZE, NULL, 1, NULL);             //create task
  xTaskCreate(MainControlTask, "MainControlTaslk", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL);  //create task
  xTaskCreate(SteeringTask, "PushButtonTaslk", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL);      //create task
  xTaskCreate(turnSignalTask, "turnSignalTask", configMINIMAL_STACK_SIZE * 2, NULL, 2, NULL);     //create task
  vTaskStartScheduler();
}


void loop() {
  //nothing in here bc free rtos
}
