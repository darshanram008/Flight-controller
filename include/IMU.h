#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <EEPROM.h>
#define EEPROM_Size 24 // 6 float values
#define MPU_ADDR 0x68
#define Accel_Sensitivity 9.8 / 4096.0 // 8g
#define Gyro_Sensitivity 131.0         // 250 deg/s
#define LED 2                          // LED pin
#define Push_Button 4                  // Push Button pin
extern float DT, Current_Time, Previous_Time;
extern float Accel_Roll, Accel_Pitch, Gyro_Roll, Gyro_Pitch;
extern float state_estimated_Roll[2][2];
extern float state_estimated_Pitch[2][2];
extern float  init_process_covar[2][2];
struct MPU6050
{
  float GX = 0;
  float GY = 0;
  float GZ = 0;
  float AX = 0;
  float AY = 0;
  float AZ = 0;
  float calib_GX = 0;
  float calib_GY = 0;
  float calib_GZ = 0;
  float calib_AX = 0;
  float calib_AY = 0;
  float calib_AZ = 0;
};
extern MPU6050 mpu_data;
void i2c_scanner();
void MPU_data();
void configure_MPU();
void calibrate_MPU(int sample_size);
void Matrix_mul(float m1[2][2], float m2[2][2], float res[2][2]);
void Matrix_transpose(float mat[2][2], float res[2][2]);
void Matrix_copy(float m1[2][2], float m2[2][2]);
void predict_state(float attitude, float rate, float state_predicted[2][2]);
void Matrix_add(float m1[2][2], float m2[2][2], float res[2][2]);
void Matrix_sub(float m1[2][2], float m2[2][2], float res[2][2]);
void Estimate_Roll(float *Roll, float *Roll_rate);
void Estimate_Pitch(float *Pitch, float *Pitch_rate);



#endif