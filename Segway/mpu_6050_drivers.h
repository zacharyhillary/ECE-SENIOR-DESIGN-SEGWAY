

#ifndef MPU_driver
#define MPU_driver

#define MPU6050_DEFAULT_ADDRESS 0x68
// Simple_MPU6050 mpu;
// double currentangle;
void mpu_setup();
void Update_Angle(int16_t *gyro, int16_t *accel, int32_t *quat);
double Get_Angle();
#endif