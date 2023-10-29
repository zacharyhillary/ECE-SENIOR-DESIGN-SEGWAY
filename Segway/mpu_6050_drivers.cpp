#include <Arduino_BuiltIn.h>

#include <math.h>
#include <avr/pgmspace.h>
#include <stdint.h>
#include <DMP_Image.h>
#include <MPU_ReadMacros.h>
#include <MPU_WriteMacros.h>
#include <Simple_MPU6050.h>

#include "mpu_6050_drivers.h"
#define MPU6050_DEFAULT_ADDRESS 0x68 // address pin low (GND), default for InvenSense evaluation board

Simple_MPU6050 mpu;

double current_angle;

void mpu_setup()
{
  current_angle = 0;
  mpu.begin();
  mpu.Set_DMP_Output_Rate_Hz(200); // Set the DMP output rate from 200Hz to 5 Minutes.
  // mpu.CalibrateAccel();
  // mpu.CalibrateGyro();
  mpu.setOffset(-2348, -1058, 826, 7, -33, 1);
  mpu.load_DMP_Image(); // Loads the DMP image into the MPU and finish configuration.
  mpu.DMP_InterruptEnable(1);
  mpu.on_FIFO(Update_Angle);
  // Set callback function that is triggered when FIFO Data is retrieved
}

void Update_Angle(int16_t *gyro, int16_t *accel, int32_t *quat)
{
  Quaternion q;
  VectorFloat gravity;
  float ypr[3] = {0, 0, 0};
  float xyz[3] = {0, 0, 0};
  mpu.GetQuaternion(&q, quat);
  mpu.GetGravity(&gravity, &q);
  mpu.GetYawPitchRoll(ypr, &q, &gravity);
  mpu.ConvertToDegrees(ypr, xyz);
  current_angle = xyz[1];
  // Serial.print("current angle:"); Serial.print(currentangle);
  // Serial.println();
}

double Get_Angle()
{
  mpu.dmp_read_fifo(0);
  return current_angle;
}
