#include "Arduino_FreeRTOS.h"
#include "mpu_6050_drivers.h"
#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>

double currentAngle;
double steering;



SoftwareSerial SWSerial(NOT_A_PIN, 16);  // RX on no pin (unused), TX on pin 11 (to S1).
SabertoothSimplified ST(SWSerial);       // We'll name the Sabertooth object ST.
                                         // For how to configure the Sabertooth, see the DIP Switch Wizard for
                                         //   http://www.dimensionengineering.com/datasheets/SabertoothDIPWizard/start.htm
                                         // Be sure to select Simplified Serial Mode for use with this library.
                                         // This sample uses a baud rate of 9600.
                                         //
                                         // Connections to make:
                                         //   Arduino TX->1  ->  Sabertooth S1
                                         //   Arduino GND    ->  Sabertooth 0V
                                         //   Arduino VIN    ->  Sabertooth 5V (OPTIONAL, if you want the Sabertooth to power the Arduino)
                                         //
                                         // If you want to use a pin other than TX->1, see the SoftwareSerial example.

void GetAngleTask(void* pvParameters) {
  int motorPower;
  while (1) {

    //xSemaphoreTake(I2CSemaphore, pdMS_TO_TICKS(100));
    currentAngle = -1 * (Get_Angle() + 7);  // set global variable to the current angle of the segway
    // if(currentAngle > 30){
    //   motorPower = 4.2*30;
    // }
    // else if(currentAngle < -30){
    //   motorPower = -4.2*30;
    // }
    // else{
    //   motorPower = 4.2*currentAngle;
    // }
    // ST.motor(1,motorPower);
    // ST.motor(2,motorPower);

    //xSemaphoreGive(I2CSemaphore);
    vTaskDelay(pdMS_TO_TICKS(16));
  }
}

struct RiderModeConfig {
  const double configKp = 9;
  const double configKi = 0;
  const double configKd = 12.5;
  const double angleOffset = 0.5;
};

struct RiderlessModeConfig {
  const double configKp = 9;
  const double configKi = 0.15;
  const double configKd = 15;
  const double angleOffset = 7.90;
};

// RiderMode riderMode = RiderMode();
// const double configKp = riderMode.configKp;
// double kp = configKp;
// const double ki = 0;
// const double configKd = riderMode.configKd;
// double kd = configKd;
// const double dt = 0;
bool riderMode = false;
int SteeringScaler(char output){
  return (0.3  - output*0.0022)*steering;
}

void MainControlTask(void* pvParameters) {
  RiderModeConfig riderModeConfig = RiderModeConfig();
  RiderlessModeConfig riderlessModeConfig = RiderlessModeConfig();
  double configKp;
  double configKd;
  double configKi;
  const double LEFT_MOTOR_SCALE = 1.05;
  bool previousRiderMode = riderMode;
  double previousError = 0;
  double integral = 0;
  int iteration = 0;
  long kpTimeStamp = millis();
  vTaskDelay(pdMS_TO_TICKS(500));
  long timerMillis = millis();
  const double slowKp = 4;
  const double slowKd = 0;
  double kp = slowKp;
  double kd = slowKd;
  double ki = 0;
  double angleOffset = riderlessModeConfig.angleOffset;


  double averager[16] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  int angleIterations = 0;
  double currentAngleOffset = 0;
  double targetAngleOffset = 0;
  double angleIncrementor = 0;

  if (riderMode) {
    configKp = riderModeConfig.configKp;
    configKd = riderModeConfig.configKd;
    configKi = riderModeConfig.configKi;
    angleOffset = riderModeConfig.angleOffset;
  } else {
    configKp = riderlessModeConfig.configKp;
    configKd = riderlessModeConfig.configKd;
    configKi = riderlessModeConfig.configKi;
    angleOffset = riderlessModeConfig.angleOffset;
  }
  bool resetConfig = true;
  while (1) {
    if (previousRiderMode != riderMode) {
      previousRiderMode = riderMode;
      resetConfig = true;
      timerMillis = millis();
      kp = slowKp;
      kd = slowKd;
      ki = 0;
      if (riderMode) {
        configKp = riderModeConfig.configKp;
        configKd = riderModeConfig.configKd;
        configKi = riderModeConfig.configKi;
        angleOffset = riderModeConfig.angleOffset;
        integral = 0;
      } else {
        configKp = riderlessModeConfig.configKp;
        configKd = riderlessModeConfig.configKd;
        configKi = riderlessModeConfig.configKi;
        angleOffset = riderlessModeConfig.angleOffset;
      }
    }
    if (millis() - timerMillis >= 1000 && resetConfig) {
      kp = configKp;
      kd = configKd;
      ki = configKi;
      resetConfig = false;
    }
    double setpoint = angleOffset;  //we want segway to balance at 0deg -> may need to tweak this value.
    double processVariable = currentAngle;

    double error = setpoint + currentAngleOffset - processVariable;
    integral += ki * error;
    double derivative = error - previousError;
    double output = kp * error + integral + kd * derivative;
    previousError = error;
    if (iteration == 8) {  // if we print every time we bog down the processor
      Serial1.print("Error:");
      Serial1.println(error);
      iteration = 0;
    }
    if (millis() - kpTimeStamp >= 5000) {
      // kd += 0.5;
      kpTimeStamp = millis();
      // Serial1.print("Kd: ");
      // Serial1.println(kd);
      // Serial1.print("current angle: ");
      // Serial1.println(currentAngle);
      // Serial1.print("Kp: ");
      // Serial1.println(kp);
    }
    iteration++;
    // Serial1.print("  kp: ");
    // Serial1.print(kp);
    // Serial1.print("  ki: ");
    // Serial1.print(ki);
    // Serial1.print("  kd: ");
    // Serial1.print(kd);
    // Serial1.print("  P+I+D:  ");
    // Serial1.print(output);
    // Serial1.print("  currentAngle:  ");
    // Serial1.print(currentAngle);
    // Serial1.print(" integral:  ");
    // Serial1.println(integral);
    // }
    // LeftMotor.setSpeed(output);
    // RightMotor.setSpeed(output):
    if (output > 127) {
      output = 127;
    }
    if (output < -127) {
      output = -127;
    }
    if (currentAngle > 45 || currentAngle < -45) {
      output = 0;
      ST.motor(1, output);
      ST.motor(2, output);
      vTaskEndScheduler();
    }

    // handle averaging
    averager[0] = averager[1];
    averager[1] = averager[2];
    averager[2] = averager[3];
    averager[3] = averager[4];
    averager[4] = averager[5];
    averager[5] = averager[6];
    averager[6] = averager[7];
    averager[7] = averager[8];
    averager[8] = averager[9];
    averager[9] = averager[10];
    averager[10] = averager[11];
    averager[11] = averager[12];
    averager[12] = averager[13];
    averager[13] = averager[14];
    averager[14] = averager[15];
    averager[15] = output;
    double sum = 0;
    for (int i = 0; i < 16; i++) {
      sum += averager[i];
    }
    double actualAverage = sum / 16.0;

    if (!riderMode) {
      if (angleIterations > 10) {
        if (!riderMode && setpoint - 2 < currentAngle && currentAngle < setpoint + 2) {
          if (actualAverage > 15) {
            angleIterations = 0;
            targetAngleOffset = 6;
            angleIncrementor = (targetAngleOffset - currentAngleOffset) / 75;
          } else if (actualAverage < -15) {
            angleIterations = 0;
            targetAngleOffset = -6;
            angleIncrementor = (targetAngleOffset - currentAngleOffset) / 75;
          } else {
            angleIterations = 0;
            targetAngleOffset = 0;
            angleIncrementor = (targetAngleOffset - currentAngleOffset) / 75;
          }
        } else {
          angleIterations = 0;
          targetAngleOffset = 0;
          angleIncrementor = (targetAngleOffset - currentAngleOffset) / 75;
        }
      }
      if (angleIterations > 75) {
        angleIterations = 0;
      }
      angleIterations++;
      currentAngleOffset += angleIterations * angleIncrementor;
    } else {
      angleIterations = 0;
      currentAngleOffset = 0;
      angleIncrementor = 0;
    }

    int leftMotorOutput;
    int rightMotorOutput;
    if (steering >= 1) {
      rightMotorOutput = output * LEFT_MOTOR_SCALE - steering;  //right turn
      leftMotorOutput = output + steering;
    } else if (steering <= -1) {                                      //left turn
      leftMotorOutput = output * LEFT_MOTOR_SCALE + steering;  //left turn
      rightMotorOutput = output - steering;
    } else {
      leftMotorOutput = output * LEFT_MOTOR_SCALE;
      rightMotorOutput = output;
    }
    ST.motor(1, leftMotorOutput);   // left motor
    ST.motor(2, rightMotorOutput);  // right motor


    vTaskDelay(1);  // ~60 Hz need to implement with different timing source if we want more precise timing
  }
}


// long lastSetTimeStamp = millis();
// void smoothlyChangeAngleOffset(double current, double next, long timePeriod) {
//   if (millis() - lastSetTimeStamp >= timePeriod) {
//     double counter = current
//     double iterator = (next - current) / 100;
//     while(counter != next) {

//     }
//   }
// }

void PushButtonTask(void* pvParameters) {
  int leftSteeringPin = A2;
  int rightSteeringPin = A3;
  pinMode(leftSteeringPin, INPUT);
  pinMode(rightSteeringPin, INPUT);
  while (1) {
    int rawRead = analogRead(leftSteeringPin);
    Serial.print("right: ");
    Serial.println(rawRead);
    double rightSteering = -1 * (((double)analogRead(rightSteeringPin) - 40.0) * (1.0 / 9.0) - 100);
    double leftSteering = -1 * (((double)analogRead(leftSteeringPin) - 14.0) * (1.0 / 9.0) - 100);
    steering = rightSteering - leftSteering;                  // ALL LEFT = -100 ALL RIGHT = 100 NONE = 0
    steering = (0.3  - output*0.0022)*steering// 0.3*steering at 0 output and and .1*steering at 90
    if ((steering <= 20) && (steering >= -20)) steering = 0;  // steering deadzone;
    /* Serial.print("Right Steering: ");
       Serial.print(rightSteering);
       Serial.print(" Left Steering: ");
       Serial.print(leftSteering);
       Serial.print(" Total Steering: ");
       Serial.println(steering);
    */
    vTaskDelay(pdMS_TO_TICKS(60));
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
    // Serial1.print("pin1: ");
    // Serial1.println(pin1);
    int pin2 = digitalRead(TURN_SIGNAL_INPUT_2);
    // Serial1.print("pin2: ");
    // Serial1.println(pin2);
    // if (pin1 == LOW) {
    // } else {
    //   riderMode = false;
    // }
    if (pin1 == LOW) {
      digitalWrite(TURN_SIGNAL_OUTPUT_1, signalLevelOne);
      signalLevelOne = !signalLevelOne;
      riderMode = true;
    } else {
      digitalWrite(TURN_SIGNAL_OUTPUT_1, HIGH);
      riderMode = false;
    }
    if (pin2 == LOW) {
      digitalWrite(TURN_SIGNAL_OUTPUT_2, signalLevelTwo);
      signalLevelTwo = !signalLevelTwo;
    } else {
      digitalWrite(TURN_SIGNAL_OUTPUT_2, HIGH);
    }
    vTaskDelay(16);  // Delay for 0.25 seconds
  }
}

void setup() {
  // put your setup code here, to run once:

  SWSerial.begin(9600);


  ST.motor(1, 0);
  ST.motor(2, 0);
  mpu_setup();


  Serial1.begin(9600);
  xTaskCreate(GetAngleTask, "GetAngleTask", configMINIMAL_STACK_SIZE, NULL, 1, NULL);             //create task
  xTaskCreate(MainControlTask, "MainControlTaslk", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL);  //create task
  xTaskCreate(PushButtonTask, "PushButtonTaslk", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL);    //create task
  xTaskCreate(turnSignalTask, "turnSignalTask", configMINIMAL_STACK_SIZE * 2, NULL, 2, NULL);     //create task
  // Serial1.println("\n\n\n STARTING RTOS.....");
  vTaskStartScheduler();
}

void loop() {
  // Test loop
  // ST.motor(1, 127);  // Go forward at full power.
  // delay(2000);       // Wait 2 seconds.
  // ST.motor(1, 0);    // Stop.
  // delay(2000);       // Wait 2 seconds.
  // ST.motor(1, -127); // Reverse at full power.
  // delay(2000);       // Wait 2 seconds.
  // ST.motor(1, 0);    // Stop.
  // delay(2000);
}
