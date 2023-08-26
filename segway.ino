#include <Arduino_FreeRTOS.h>
#include <mpu_6050_drivers.h>

double currentAngle;
TaskHandle_t Handle_Get_Angle_Task;

void setup() {
  Serial.begin(115200);
  mpu_setup();
  xTaskCreate(Get_Angle,     "Get_Angle",       256, NULL, tskIDLE_PRIORITY + 2, &Handle_Get_Angle_Task);//create task
  vTaskStartScheduler();//start rtos
}

static void Get_Angle(void* pvParameters){
  while(1){
    currentAngle = Get_Angle();// set global variable to the current angle of the segway
    Serial.print(currentAngle);Serial.print("\n");

}78

}

void loop() {

}

