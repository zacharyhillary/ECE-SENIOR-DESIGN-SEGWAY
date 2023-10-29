#include <Arduino_FreeRTOS.h>
#include "mpu_6050_drivers.h"
double currentAngle;

void GetAngleTask(void* pvParameters) {
  while (1) {

    //xSemaphoreTake(I2CSemaphore, pdMS_TO_TICKS(100));
    currentAngle = Get_Angle();  // set global variable to the current angle of the segway
    //xSemaphoreGive(I2CSemaphore);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
void MainControlTask(void* pvParameters) {
  double previousError = 0;
  double integral = 0;
  double kp=4;
  double ki=0;
  double kd=0;
  double dt=0;
  
  while (1) {
    double setpoint = 0;  //we want segway to balance at 0deg -> may need to tweak this value.
    double processVariable = currentAngle;

    double error = setpoint - processVariable;
    integral += ki * error;
    double derivative = error - previousError;
    double output = kp * error + integral + kd * derivative;
    previousError = error;
    Serial.print("  kp: ");
    Serial.print(kp);
    Serial.print("  ki: ");
    Serial.print(ki);
    Serial.print("  kd: ");
    Serial.print(kd);
    Serial.print("  P+I+D:  ");
    Serial.print(output);
    Serial.print("  currentAngle:  ");
    Serial.println(currentAngle);

    
    // LeftMotor.setSpeed(output);
    // RightMotor.setSpeed(output):
    vTaskDelay(pdMS_TO_TICKS(100));  // ~60 Hz need to implement with different timing source if we want more precise timing
  }
}


void setup() {
  // put your setup code here, to run once:
 Serial.begin(38400);
 mpu_setup();
  xTaskCreate(GetAngleTask, "GetAngleTask", configMINIMAL_STACK_SIZE / 2, NULL, 1, NULL);  //create task
  xTaskCreate(MainControlTask, "MainControlTaslk", configMINIMAL_STACK_SIZE , NULL, 1, NULL);  //create task
  Serial.println("\n\n\n STARTING RTOS.....");

}

void loop() {
 
}
