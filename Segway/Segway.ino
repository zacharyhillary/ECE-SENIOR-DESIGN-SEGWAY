#include "Arduino_FreeRTOS.h"
#include "mpu_6050_drivers.h"
#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>
#include <Adafruit_GFX.h>
#include <Adafruit_FT6206.h>
#include <Adafruit_ILI9341.h>
#include "semphr.h"
#include "splash_screen.h"
#include <task.h>

//screen pins
#define TFT_CLK 52
#define TFT_MISO 50
#define TFT_MOSI 51
#define TFT_CS 10
//#define TFT_RST 8
#define TFT_DC 9

// Common 16-bit color values:
#define BLACK 0x0000
#define BLUE 0x001F
#define RED 0xF800
#define GREEN 0x07E0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF




Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

double currentAngle;
double currentScreenAngle = 0;
double steering;

double displayAngle;


#define TURN_SIGNAL_OUTPUT_1 26
#define TURN_SIGNAL_OUTPUT_2 27
#define TURN_SIGNAL_INPUT_1 28
#define TURN_SIGNAL_INPUT_2 29

SoftwareSerial SWSerial(NOT_A_PIN, 16);  // RX on no pin (unused), TX on pin 11 (to S1).
SabertoothSimplified ST(SWSerial);       // We'll name the Sabertooth object ST.

void pushToArray(int* arrayOfInputs, int nextInput, int arrayLength) {
  int index = 0;
  while(index < arrayLength - 1) {
    arrayOfInputs[index] = arrayOfInputs[index+1];
    index++;
  }
  arrayOfInputs[index] = nextInput;
}

void pushToDoubleArray(double* arrayOfInputs, double nextInput, int arrayLength) {
  int index = 0;
  while(index < arrayLength - 1) { // 0, 1, 2, 3 -> 4
    arrayOfInputs[index] = arrayOfInputs[index+1];
    index++;
  }
  arrayOfInputs[index] = nextInput; // 4 reassigned down here
}

bool checkEachLessThan(int* array, int value, int arrayLength) {
    int index = 0;
    while (index < arrayLength) {
      if (array[index] >= value) {
        return false;
      }
      index++;
    }
    return true;
}

bool checkEachGreaterThan(int* array, int value, int arrayLength) {
    int index = 0;
    while (index < arrayLength) {
      if (array[index] <= value) {
        return false;
      }
      index++;
    }
    return true;
}

void clearIntArray(int* array, int arrayLength) {
  for(int i=0; i<arrayLength; i++) {
    array[i] = 0;
  }
}

void GetAngleTask(void* pvParameters) {
  int motorPower;
  while (1) {
    double temp = -1 * (Get_Angle() + 7);
    currentAngle = temp;  // set global variable to the current angle of the segway
    currentScreenAngle = temp;
    vTaskDelay(pdMS_TO_TICKS(10));
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

void handleSetValueSmoothing(double* targetValue, double* effectiveValue, double change) {
  if (*effectiveValue > *targetValue) {
    *effectiveValue = max(*effectiveValue - change, *targetValue);
  } else if (*effectiveValue < *targetValue) {
    *effectiveValue = min(*effectiveValue + change, *targetValue);
  }
}

double output;
void MainControlTask(void* pvParameters) {

  bool previousRiderMode = !riderMode;  // SET IT so it goes into if statement on first loop
  vTaskDelay(pdMS_TO_TICKS(500));       // LET IMU STABILISE

  const double slowKp = 4;
  const double slowKd = 0;

  const double riderKp = 9;
  const double riderKi = 0;
  const double riderKd = 12.5;
  const double riderSetpoint = 0;

  const double boundedRiderKp = 9;
  const double boundedRiderKi = 0.1;
  const double boundedRiderKd = 12.5;

  const double noRiderKp = 9;
  const double noRiderKi = 0.15;
  const double noRiderKd = 15;
  const double noRiderSetpoint = 7.4;

  const double LEFT_MOTOR_SCALE = 1;  // 10 percent increase

  double effectiveKp = 0;
  double targetKp;
  double ki;
  double kd;
  bool resetConfig;
  double targetSetpoint;
  double effectiveSetpoint = currentAngle;

  int lastOutputs[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  long recordTimer = millis();
  bool overrideOutput = false;
  int overrideTarget = 60;
  long overrideStart = millis();
  long overrideDuration = 1000;
  long overrideChange = 1.5;

  const double smoothAngleChange = 0.1;
  const double smoothKpChange = 0.25;

  while (1) {
    if (previousRiderMode != riderMode) {  //gets activated when you switch rider mode AND FIRST TIME BOOT UP
      previousRiderMode = riderMode;
      integral = 0;
      if (riderMode) {
        targetSetpoint = riderSetpoint;
        targetKp = riderKp;
        kd = riderKd;
        ki = riderKi;
      } else {
        targetSetpoint = noRiderSetpoint;
        targetKp = noRiderKp;
        kd = noRiderKd;
        ki = noRiderKi;
      }  //if rider is not present
    }
    
    if(riderMode && (currentAngle > 3 || currentAngle < -3)) {//increase Ki past certain angle thresholds to keep in bounded angles
      targetKp = boundedRiderKp;
      ki = boundedRiderKi;
      kd = boundedRiderKd;
    } else if (riderMode) {
      integral *= 0.98;
      targetKp = riderKp;
      ki = riderKi;
      kd = riderKd;
    }
    handleSetValueSmoothing(&targetSetpoint, &effectiveSetpoint, smoothAngleChange);
    handleSetValueSmoothing(&targetKp, &effectiveKp, smoothKpChange);

    output = ComputePID(effectiveKp, ki, kd, effectiveSetpoint, currentAngle);

    if (output > 127) {  //bound the output
      output = 127;
    }
    if (output < -127) {  //bound the output
      output = -127;
    }

    if (currentAngle > 50 || currentAngle < -40) {  // SAFETY CODE TO SHUT DOWN SEGWAY in case extreme angle
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

    ST.motor(1, leftMotorOutput * LEFT_MOTOR_SCALE);  // left motor
    ST.motor(2, rightMotorOutput);                    // right motor

    vTaskDelay(pdMS_TO_TICKS(10));  // 100 Hz
  }
}

void SteeringTask(void* pvParameters) {
  int leftSteeringPin = A2;
  int rightSteeringPin = A3;
  int targetSteering = 0;
  int currentSteering = 0;
  int lastThreeInputs[] = { 0, 0, 0 };
  pinMode(leftSteeringPin, INPUT);
  pinMode(rightSteeringPin, INPUT);
  while (1) {
    double rightSteeringRaw = analogRead(rightSteeringPin);
    double leftSteeringRaw = analogRead(leftSteeringPin);
    if (rightSteeringRaw < 700 && leftSteeringRaw > 700) {
      targetSteering = 100;
      pushToArray(lastThreeInputs, 100, 3);
    } else if (rightSteeringRaw > 700 && leftSteeringRaw < 700) {
      targetSteering = -100;
      pushToArray(lastThreeInputs, -100, 3);
    } else {
      targetSteering = 0;
      pushToArray(lastThreeInputs, 0, 3);
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



    steering = (0.3 - output * 0.0022) * currentSteering;  // 0.3*steering at 0 output and and .1*steering at 90
    //if ((steering <= 20) && (steering >= -20)) steering = 0;  // steering deadzone;
    vTaskDelay(pdMS_TO_TICKS(75));
  }
}

void RiderModeTask(void* pvParameters) {
  // pinMode(TURN_SIGNAL_OUTPUT_1, OUTPUT);
  
  const int RIDER_MODE_SWITCH_PIN = 8;  //connects to the trigger pin on the distance sensor
  pinMode(RIDER_MODE_SWITCH_PIN, INPUT_PULLUP);
  //int distance;
  //int distanceArray[6] = { 50, 50, 50, 50, 50, 50 };
  
  while (1) {
    //float echoTime;  //variable to store the time it takes for a ping to bounce off an object         //variable to store the distance calculated from the echo time
    //send out an ultrasonic pulse that's 10ms long
      riderMode = digitalRead(RIDER_MODE_SWITCH_PIN); 
    // pinMode(trigEchoPin, OUTPUT);
    // digitalWrite(trigEchoPin, HIGH);
    // delayMicroseconds(10);
    // digitalWrite(trigEchoPin, LOW);
    // pinMode(trigEchoPin, INPUT);
    //echoTime = pulseInLong(trigEchoPin, HIGH);  //use the pulsein command to see how long it takes for the pulse to bounce back to the sensor

    //distance = echoTime / 148.0;  //calculate the distance of the object that reflected the pulse (half the bounce time multiplied by the speed of sound)
    //pushToArray(distanceArray, distance, 6);

    // if (checkEachGreaterThan(distanceArray, 20, 6)) {
    //   riderMode = false;
    // } else if (checkEachLessThan(distanceArray, 20, 6)) {
    //   riderMode = true;
    // }
    // digitalWrite(TURN_SIGNAL_OUTPUT_1, riderMode);
    vTaskDelay(pdMS_TO_TICKS(250));
  }
}

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
      digitalWrite(TURN_SIGNAL_OUTPUT_1, signalLevelOne);
      signalLevelOne = !signalLevelOne;
    } else {
      digitalWrite(TURN_SIGNAL_OUTPUT_1, HIGH);
    }
    if (pin2 == LOW) {
      digitalWrite(TURN_SIGNAL_OUTPUT_2, signalLevelTwo);
      signalLevelTwo = !signalLevelTwo;
    } else {
      digitalWrite(TURN_SIGNAL_OUTPUT_2, HIGH);
    }
    // Serial1.print("Current Angle: ");
    // Serial1.println(currentAngle);
    // Serial1.print("Output: ");
    // Serial1.println(output);
    vTaskDelay(pdMS_TO_TICKS(250));  // Delay for 0.25 seconds
  }
}

void batteryDebug(int raw, double voltage, double batt) {
  Serial1.print("Raw: ");
  Serial1.print(raw);
  Serial1.print(" | Input: ");
  Serial1.print(voltage);
  Serial1.print("[v] | ");
  Serial1.print(" Batt: ");
  Serial1.print(batt);
  Serial1.println("[v]");
}

double avgOfDoubleArray(double* arrayOfInputs, int arrayLength) {
  double sum = 0.0;
  for (int i=0; i<arrayLength; i++) {
    sum = sum + arrayOfInputs[i];
  }
  return sum / arrayLength;
}

//battery level
double percent_battery;
void batteryLevelTask() {
  pinMode(A1, INPUT);
  double battVals[5] = { 50.0, 50.0, 50.0, 50.0, 50.0 };
  const double BATTERY_DEAD = 21.0;
  const double BATTERY_MAX = 26.0;
  while(1) {
      int raw_data_in = analogRead(A1);  // read the input pin
      double voltage_input = 5.0 * raw_data_in / 1024.0;
      double voltage_battery = voltage_input * 24.2 / 2.19;
      double result = (voltage_battery - BATTERY_DEAD) * 100.0 / (BATTERY_MAX - BATTERY_DEAD);
      if (voltage_battery < BATTERY_DEAD) {
        result = 0.0;
      } else if (voltage_battery > BATTERY_MAX) {
        result = 100.0;
      }
      pushToDoubleArray(battVals, result, 5);
      percent_battery = avgOfDoubleArray(battVals, 5);
      // batteryDebug(raw_data_in, voltage_input, voltage_battery);  // print values to serial port
      vTaskDelay(pdMS_TO_TICKS(500));
  }
}

int runtime = 0;
void runtimeTask(){
  while(1){
    runtime++;
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

#define FRAME_W 150
#define FRAME_H 40
#define angleDisplayX  200
#define angleDisplayY  40
#define battDisplayX 230
#define battDisplayY 95
#define riderDisplayX 230
#define riderDisplayY 140
#define runtimeDisplayX 200
#define runtimeDisplayY 195



void updateScreenTask(void* pvParameters) {

  double displayAngle = 0;
  double displayBattLevel = 0.0;
  int displayRuntime = 0;
  bool displayRiderMode = false;
  double tempAngle = 0;
  double tempBattLevel = 0.0;
  int tempRuntime = 0;
  bool tempRiderMode = false;
  bool initialPrint = false;
  // char *prevPrint;

  while (1) {
    
    //display battery %
    tempBattLevel = percent_battery;
    if(tempBattLevel != displayBattLevel || !initialPrint) { //refresh only when battery level changes
      displayBattLevel = tempBattLevel;
      char printText[5];
      tft.fillRect(battDisplayX, battDisplayY, FRAME_W, FRAME_H, BLACK);
      vTaskDelay(pdMS_TO_TICKS(1));
      tft.setCursor(battDisplayX, battDisplayY); //adjust to your liking
      dtostrf(displayBattLevel, 4, 2, printText);
      tft.print(printText);
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    //display tilt angle
    tempAngle = currentScreenAngle;
    if (tempAngle != displayAngle) { //refresh only when angle changes
      char newPrint[4];
      displayAngle = tempAngle;
      dtostrf(displayAngle, 4, 2, newPrint);

      tft.fillRect(angleDisplayX, angleDisplayY, FRAME_W, FRAME_H, BLACK);
      vTaskDelay(pdMS_TO_TICKS(1));
      tft.setCursor(angleDisplayX, angleDisplayY); // adjust to your liking // y- 0 is top // x - 0 is left
      tft.print(newPrint);
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    // //display Rider Mode
    tempRiderMode = riderMode;// might have to do the above semaphore on this as well, but might be ok
    if (tempRiderMode != displayRiderMode || !initialPrint) { //refresh only when angle changes
      displayRiderMode = tempRiderMode;
      tft.fillRect(riderDisplayX, riderDisplayY, FRAME_W, FRAME_H, BLACK);
      vTaskDelay(pdMS_TO_TICKS(1));
      tft.setCursor(riderDisplayX, riderDisplayY); //adjust to your liking
      if (displayRiderMode) {
        tft.print("Yes");
      } else {
        tft.print("No");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    //display runtime
    tempRuntime = runtime;
    if(tempRuntime != displayRuntime){ //refresh only when time changes
      displayRuntime = tempRuntime;
      tft.fillRect(runtimeDisplayX, runtimeDisplayY, FRAME_W, FRAME_H, BLACK);
      vTaskDelay(pdMS_TO_TICKS(1));
      tft.setCursor(runtimeDisplayX, runtimeDisplayY); //adjust to your liking
      tft.print(displayRuntime);
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    //display speed?

    if (!initialPrint) initialPrint = true;
  }
}

const int screenWidth = 320;
const int screenHeight = 240;
void splashScreenDisplay(){
  tft.drawBitmap(0, 0, cougarBMP, screenHeight, screenWidth, 0xFFFF, 0xF800);
}


void setup() {
  SWSerial.begin(9600);
  ST.motor(1, 0);
  ST.motor(2, 0);

  //screen init
  tft.begin();
  tft.setRotation(2);
  splashScreenDisplay();
  tft.setRotation(3);
  //delay(5000);
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextSize(3);
  tft.setCursor(angleDisplayX - 150, angleDisplayY);
  tft.print("Angle: ");
  tft.setCursor(battDisplayX - 180, battDisplayY);
  tft.print("Batt [%]: ");
  tft.setCursor(runtimeDisplayX - 150, runtimeDisplayY);
  tft.print("Runtime: ");
  tft.setCursor(riderDisplayX - 180, riderDisplayY);
  tft.print("Ridermode: ");

  mpu_setup();
  Serial1.begin(9600);
  Serial1.println("starting rtos....");
  xTaskCreate(GetAngleTask, "gat", configMINIMAL_STACK_SIZE, NULL, 1, NULL);             //create task
  xTaskCreate(MainControlTask, "mct", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL);  //create task
  xTaskCreate(SteeringTask, "pbt", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL);      //create task
  xTaskCreate(turnSignalTask, "tst", configMINIMAL_STACK_SIZE * 2, NULL, 2, NULL);
  xTaskCreate(RiderModeTask, "rmt", configMINIMAL_STACK_SIZE, NULL, 3, NULL);  //create task
  xTaskCreate(runtimeTask, "clk", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  xTaskCreate(batteryLevelTask, "blt", configMINIMAL_STACK_SIZE * 2, NULL, 3, NULL);
  // xTaskCreate(updateScreenTask, "ust", configMINIMAL_STACK_SIZE * 15, NULL, 3, NULL);
  vTaskStartScheduler();
}


void loop() {
  //nothing in here bc free rtos
}
