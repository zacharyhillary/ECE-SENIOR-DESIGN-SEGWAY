#include "Arduino_FreeRTOS.h"
#include "mpu_6050_drivers.h"
#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>

double currentAngle;
double steering;



SoftwareSerial SWSerial(NOT_A_PIN, 16); // RX on no pin (unused), TX on pin 11 (to S1).
SabertoothSimplified ST(SWSerial); // We'll name the Sabertooth object ST.
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
    currentAngle = -1*(Get_Angle()+7);  // set global variable to the current angle of the segway
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



const double configKp = 9;
double kp = configKp;
const double ki = 0;
const double configKd = 12.5;
double kd = configKd;
const double dt = 0;
void MainControlTask(void* pvParameters) {
  double previousError = 0;
  double integral = 0;
  char iteration = 0;
  vTaskDelay(pdMS_TO_TICKS(500));
  long initialMillis = millis();
  kp = 2;
  kd = 0;
  while (1) {
    if (millis() - initialMillis >= 1000) {
      kp = configKp;
      kd = configKd;
    }
    double setpoint = 0;  //we want segway to balance at 0deg -> may need to tweak this value.
    double processVariable = currentAngle;

    double error = setpoint - processVariable;
    integral += ki * error;
    double derivative = error - previousError;
    double output = kp * error + integral + kd * derivative;
    previousError = error;
    // if (iteration == 25) {  // if we print every time we bog down the processor
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
    //   iteration = 0;
    // }
    // iteration++;
    // LeftMotor.setSpeed(output);
    // RightMotor.setSpeed(output):
    if(output > 127){
      output = 127;
    }
    if(output < -127){
      output = -80;
    }
    if(currentAngle > 45 || currentAngle < -45){
      output = 0;
      ST.motor(1, output);
      ST.motor(2, output);
      //vTaskEndScheduler();
    }
    int leftMotorOutput;
    int rightMotorOutput;
    if(steering >=1){
      rightMotorOutput = output-0.15*steering;//right turn
      leftMotorOutput = output+0.15*steering;
    }
    else if(steering <=-1){//left turn
      leftMotorOutput = output + 0.15*steering;//left turn
      rightMotorOutput = output-0.15*steering;
    }
    else {
      leftMotorOutput = output;
      rightMotorOutput = output;
    }
    ST.motor(1, leftMotorOutput);//left motor
    ST.motor(2, rightMotorOutput);//right motor
    
    vTaskDelay(1);  // ~60 Hz need to implement with different timing source if we want more precise timing
  }
}


void PushButtonTask(void* pvParameters){
  int leftSteeringPin=A2;
  int rightSteeringPin=A3;
  pinMode(leftSteeringPin, INPUT); 
  pinMode(rightSteeringPin, INPUT); 
  while(1){
    int rawRead = analogRead(leftSteeringPin);
    Serial.print("right: ");
    Serial.println(rawRead);
    double rightSteering = -1*(((double)analogRead(rightSteeringPin)-40.0 )*(1.0/9.0)-100);
    double leftSteering = -1*(((double)analogRead(leftSteeringPin)-14.0 )*(1.0/9.0)-100);
    steering = rightSteering - leftSteering;// ALL LEFT = -100 ALL RIGHT = 100 NONE = 0
    if((steering<=20) && (steering >=-20))steering=0;// steering deadzone;
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
void turnSignalTask(void *pvParameters) {
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
    vTaskDelay(16);  // Delay for 0.25 seconds
  }
}

void setup() {
  // put your setup code here, to run once:
  
  SWSerial.begin(9600);
  

  ST.motor(1,0);
  ST.motor(2,0);
  mpu_setup();
  

  Serial1.begin(9600);
  xTaskCreate(GetAngleTask, "GetAngleTask", configMINIMAL_STACK_SIZE, NULL, 1, NULL);         //create task
  xTaskCreate(MainControlTask, "MainControlTaslk", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL);  //create task
  xTaskCreate(PushButtonTask, "PushButtonTaslk", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL);  //create task
  xTaskCreate(turnSignalTask, "turnSignalTask", configMINIMAL_STACK_SIZE * 2, NULL, 2 , NULL); //create task
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
