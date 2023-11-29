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
double steering;

double displayAngle;


#define TURN_SIGNAL_OUTPUT_1 26
#define TURN_SIGNAL_OUTPUT_2 27
#define TURN_SIGNAL_INPUT_1 28
#define TURN_SIGNAL_INPUT_2 29

SoftwareSerial SWSerial(NOT_A_PIN, 16);  // RX on no pin (unused), TX on pin 11 (to S1).
SabertoothSimplified ST(SWSerial);       // We'll name the Sabertooth object ST.

SemaphoreHandle_t batterySemaphore;
SemaphoreHandle_t angleSemaphore;
SemaphoreHandle_t runtimeSemaphore;
SemaphoreHandle_t riderModeSemaphore;



void GetAngleTask(void* pvParameters) {
  int motorPower;
  while (1) {
    //if (xSemaphoreTake(angleSemaphore, pdMS_TO_TICKS(10)) == pdTRUE){
    currentAngle = -1 * (Get_Angle() + 7);  // set global variable to the current angle of the segway
    //Serial1.print("Current Angle: ");
    //Serial1.println(currentAngle);


    //xSemaphoreGive(angleSemaphore);
    //}
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

  const double boundedRiderKp = 9;
  const double boundedRiderKi = 0.15;
  const double boundedRiderKd = 12.5;

  const double noRiderKp = 9;
  const double noRiderKi = 0.15;
  const double noRiderKd = 15;
  const double noRiderSetpoint = 7.4;

  const double boundedNoRiderKp = 9;
  const double boundedNoRiderKi = 0.15;
  const double boundedNoRiderKd = 12.5;

  const double LEFT_MOTOR_SCALE = 1;  // 10 percent increase

  double kp;
  double ki;
  double kd;
  bool resetConfig;
  double setpoint;

  // long previousMillis = millis();
  // long currentMillis = millis();

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

    if((currentAngle > 3 || currentAngle < -3) && riderMode){//increase Ki past certain angle thresholds to keep in bounded angles
      kp = boundedRiderKp;
      ki = boundedRiderKi;
      kd = boundedRiderKd;
    }
    else if (riderMode){
      integral *=0.9;
      kp = riderKp;
      ki = riderKi;
      kd = riderKd;
    }

   // if (xSemaphoreTake(angleSemaphore, pdMS_TO_TICKS(15)) == pdTRUE)//semaphore to protect currentAngle 
    //{
    output = ComputePID(kp, ki, kd, setpoint, currentAngle);
    

    //xSemaphoreGive(angleSemaphore);
    //}

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
    
    // currentMillis = millis();
    // Serial.print("Difference:  |  ");
    // Serial.println(currentMillis - previousMillis);
    // previousMillis = currentMillis;

    vTaskDelay(pdMS_TO_TICKS(10));  // ~60 Hz need to implement with different timing source if we want faster timing
  }
}


enum SteeringDirection {
  STEERING_LEFT,
  STEERING_RIGHT,
  NONE
};

void pushToArray(int* arrayOfInputs, int nextInput, int arrayLength) {
  int index = 0;
  while(index < arrayLength - 1) {
    arrayOfInputs[index] = arrayOfInputs[index+1];
    index++;
  }
  arrayOfInputs[index] = nextInput;
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

void RiderModeTask(void* pvParameters) {
  // pinMode(TURN_SIGNAL_OUTPUT_1, OUTPUT);
  
  const int RIDER_MODE_SWITCH_PIN = 8;  //connects to the trigger pin on the distance sensor
  pinMode(RIDER_MODE_SWITCH_PIN, INPUT_PULLUP);
  //int distance;
  //int distanceArray[6] = { 50, 50, 50, 50, 50, 50 };
  
  while (1) {
    //float echoTime;  //variable to store the time it takes for a ping to bounce off an object         //variable to store the distance calculated from the echo time
    //send out an ultrasonic pulse that's 10ms long
    //if(xSemaphoreTake(riderModeSemaphore, pdMS_TO_TICKS(250)) == pdTRUE){
        riderMode = digitalRead(RIDER_MODE_SWITCH_PIN); 
      //  xSemaphoreGive(riderModeSemaphore);
    //}
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
    // Serial.print("Rider mode is: ");
    // Serial.println(riderMode);
    // Serial.print("Distance is: ");
    // Serial.println(distance);
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
    Serial1.print("Current Angle: ");
    Serial1.println(currentAngle);
    Serial1.print("Output: ");
    Serial1.println(output);
    vTaskDelay(pdMS_TO_TICKS(250));  // Delay for 0.25 seconds
  }
}

//battery level
int percent_battery;
void batteryLevelTask() {
  pinMode(A2, INPUT);
  analogReference(INTERNAL2V56);
  while(1) {
    //if ((xSemaphoreTake(batterySemaphore, pdMS_TO_TICKS(50)) == pdTRUE)){//wait 50 ms for other screen tasks to finish 
      int raw_data_in = analogRead(A2);  // read the input pin
      double voltage_input = 5.0 * raw_data_in / 1024.0;
      double voltage_battery = voltage_input * 15.523;
      percent_battery = voltage_battery * (121.0 / 26.0) - 21;
      if (voltage_battery < 21) percent_battery = 0;
    //batteryDebug(raw_data_in, voltage_input, voltage_battery);  // print values to serial port
   // xSemaphoreGive(batterySemaphore);
    //}
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}



int runtime = 0;
void runtimeTask(){
  while(1){
    //if(xSemaphoreTake(runtimeSemaphore, pdMS_TO_TICKS(100)) == pdTRUE){
    runtime++;
    //xSemaphoreGive(runtimeSemaphore);
    //}
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

#define FRAME_W 150
#define FRAME_H 50
#define angleDisplayX  235
#define angleDiplayY  40
#define battDisplayX 235
#define battDisplayY 60
#define riderDisplayX 235
#define riderDisplayY 80
#define runtimeDisplayX 235
#define runtimeDisplayY 100



void updateScreenTask(void* pvParameters) {
  bool touchInProgress = false;
  int displayAngle = 0;
  int displayBattLevel = 0;
  int displayRuntime = 0;
  bool displayRiderMode = false;
  int tempAngle = 0;
  int tempBattLevel = 0;
  int tempRuntime = 0;
  bool tempRiderMode = false;
  
  while (1) {
    
    //display battery %
    //if(xSemaphoreTake(batterySemaphore, pdMS_TO_TICKS(50)) == pdTRUE){
      tempBattLevel = percent_battery;
      //xSemaphoreGive(batterySemaphore);
    //}
    if(tempBattLevel != displayBattLevel){//refresh only when battery level changes
      displayBattLevel = tempBattLevel;
      tft.fillRect(battDisplayX, battDisplayY, FRAME_W, FRAME_H, RED);
      tft.setCursor(battDisplayX, battDisplayY); //adjust to your liking
      tft.setTextSize(3);
      tft.print(displayBattLevel);
    }

    //display tilt angle
    //if(xSemaphoreTake(angleSemaphore, pdMS_TO_TICKS(50)) == pdTRUE){//added semaphores to prevent corruption of currentAngle
      tempAngle = currentAngle;
      //xSemaphoreGive(angleSemaphore);
    //}
    if(tempAngle != displayAngle){//refresh only when angle changes
      displayAngle = tempAngle;
      tft.fillRect(angleDisplayX, angleDiplayY, FRAME_W, FRAME_H, BLACK);
      tft.setCursor(angleDisplayX, angleDiplayY); //adjust to your liking
      tft.setTextSize(3);
      tft.print(displayAngle);
      //Serial1.println(displayAngle);
    }

    //display Rider Mode
    //if(xSemaphoreTake(riderModeSemaphore, pdMS_TO_TICKS(50))==pdTRUE){
      tempRiderMode = riderMode;// might have to do the above semaphore on this as well, but might be ok
      //xSemaphoreGive(riderModeSemaphore);
    //}
    
    if(tempRiderMode != displayRiderMode){//refresh only when angle changes
      displayRiderMode = tempRiderMode;
      tft.fillRect(riderDisplayX, riderDisplayY, FRAME_W, FRAME_H, BLACK);
      tft.setCursor(riderDisplayX, riderDisplayY); //adjust to your liking
      tft.setTextSize(3);
      tft.print(displayRiderMode);
    }

    //display runtime
    //if(xSemaphoreTake(runtimeSemaphore, pdMS_TO_TICKS(50)) == pdTRUE){//added semaphores to prevent corruption of currentAngle
      tempRuntime = runtime;
      //xSemaphoreGive(runtimeSemaphore);
    //}
    if(tempRuntime != displayRuntime){//refresh only when time changes
      displayRuntime = tempRuntime;
      tft.fillRect(runtimeDisplayX, runtimeDisplayY, FRAME_W, FRAME_H, BLACK);
      tft.setCursor(runtimeDisplayX, runtimeDisplayY); //adjust to your liking
      tft.setTextSize(3);
      tft.print(displayRuntime);
    }



    //display speed?

  
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

const int screenWidth = 320;
const int screenHeight = 240;
void splashScreenDisplay(){
  tft.drawBitmap(0, 0, cougarBMP, screenHeight, screenWidth, 0xFFFF, 0xF800);
  //Serial.println("Display finished");
}


void setup() {
  SWSerial.begin(9600);
  ST.motor(1, 0);
  ST.motor(2, 0);

  //screen init
  tft.begin();
  tft.setRotation(2);
  splashScreenDisplay();
  //delay(5000);
  tft.fillScreen(ILI9341_BLACK);

  mpu_setup();
  Serial1.begin(9600);
  Serial1.println("starting rtos....");
  xTaskCreate(GetAngleTask, "gat", configMINIMAL_STACK_SIZE, NULL, 1, NULL);             //create task
  xTaskCreate(MainControlTask, "mct", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL);  //create task
  xTaskCreate(SteeringTask, "pbt", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL);      //create task
  xTaskCreate(turnSignalTask, "tst", configMINIMAL_STACK_SIZE * 2, NULL, 2, NULL);
  xTaskCreate(RiderModeTask, "rmt", configMINIMAL_STACK_SIZE, NULL, 3, NULL);  //create task
  xTaskCreate(runtimeTask, "clk", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  xTaskCreate(updateScreenTask, "ust", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
  vTaskStartScheduler();
}


void loop() {
  //nothing in here bc free rtos
}
