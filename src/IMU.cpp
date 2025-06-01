#include <IMU.h>
float Accel_Roll = 0;
float Accel_Pitch = 0;
float Gyro_Roll = 0;
float Gyro_Pitch = 0;
float  init_process_covar[2][2] = {{0.5, 0}, {0, 0.1}};
float DT, Current_Time, Previous_Time;
float state_estimated_Roll[2][2];
float state_estimated_Pitch[2][2];
MPU6050 mpu_data;
void i2c_scanner()
{
  byte error, address;
  int nDevices;
  Serial.begin(9600);
  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++)
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}
void MPU_data()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14);                        // increment and request the address from the next 14 bytes 3B to 48
  int16_t raw_Accelx = (Wire.read() << 8 | Wire.read()); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  int16_t raw_Accely = (Wire.read() << 8 | Wire.read()); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  int16_t raw_Accelz = (Wire.read() << 8 | Wire.read()); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  int16_t raw_Temp = (Wire.read() << 8 | Wire.read());   // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  int16_t raw_Gyrox = (Wire.read() << 8 | Wire.read());  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  int16_t raw_Gyroy = (Wire.read() << 8 | Wire.read());  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  int16_t raw_Gyroz = (Wire.read() << 8 | Wire.read());  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  mpu_data.AX = (raw_Accelx * Accel_Sensitivity);        // Convert to m/s²
  mpu_data.AY = (raw_Accely * Accel_Sensitivity);
  mpu_data.AZ = (raw_Accelz * Accel_Sensitivity);
  mpu_data.GX = (raw_Gyrox / Gyro_Sensitivity); // Convert to deg/s
  mpu_data.GY = (raw_Gyroy / Gyro_Sensitivity);
  mpu_data.GZ = (raw_Gyroz / Gyro_Sensitivity);
}
void configure_MPU()
{

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0x00); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); // GYRO_CONFIG register
  Wire.write(0x00); // set to 250 degrees per second
  Wire.endTransmission();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); // ACCEL_CONFIG register
  Wire.write(0x10); // set to 8g
  Wire.endTransmission();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A); // DLPF filter
  Wire.write(0x02); // set to 94 Hz bandwidth
  Wire.endTransmission();
  mpu_data.calib_AX = EEPROM.readFloat(0);
  mpu_data.calib_AY = EEPROM.readFloat(4);
  mpu_data.calib_AZ = EEPROM.readFloat(8);
  mpu_data.calib_GX = EEPROM.readFloat(12);
  mpu_data.calib_GY = EEPROM.readFloat(16);
  mpu_data.calib_GZ = EEPROM.readFloat(20);
  MPU_data();
  float AccelX = mpu_data.AX - mpu_data.calib_AX;
  float AccelY = mpu_data.AY - mpu_data.calib_AY;
  float AccelZ = mpu_data.AZ - mpu_data.calib_AZ;
  Accel_Roll = atan2(AccelY, AccelZ) * 180 / M_PI;
  Accel_Pitch = atan2(AccelX, sqrt(AccelY * AccelY + AccelZ * AccelZ)) * 180 / M_PI;
  state_estimated_Roll[0][0] = Accel_Roll; // roll
  state_estimated_Roll[1][0] = 0;          // roll rate
   state_estimated_Pitch[0][0] = Accel_Pitch; // pitch
  state_estimated_Pitch[1][0] = 0;          // pitch rate
  Gyro_Roll = Accel_Roll;
  Gyro_Pitch = Accel_Pitch;
}
void calibrate_MPU(int sample_size)
{

  digitalWrite(LED, HIGH);
  float Sum_Ax = 0.0, Sum_Ay = 0.0, Sum_Az = 0.0, Sum_Gx = 0.0, Sum_Gy = 0.0, Sum_Gz = 0.0;
  for (int i = 0; i <= 50; i++)
  {
    MPU_data(); // ignore first 50 samples
    delay(5);
  }
  for (int i = 0; i < sample_size; i++)
  {
    MPU_data();
    Sum_Ax += mpu_data.AX;
    Sum_Ay += mpu_data.AY;
    Sum_Az += mpu_data.AZ;
    Sum_Gx += mpu_data.GX;
    Sum_Gy += mpu_data.GY;
    Sum_Gz += mpu_data.GZ;
    delay(2);
  }
  mpu_data.calib_AX = Sum_Ax / sample_size;
  mpu_data.calib_AY = Sum_Ay / sample_size;
  mpu_data.calib_AZ = (Sum_Az / sample_size) - 9.81;
  mpu_data.calib_GX = Sum_Gx / sample_size;
  mpu_data.calib_GY = Sum_Gy / sample_size;
  mpu_data.calib_GZ = Sum_Gz / sample_size;

  EEPROM.writeFloat(0, mpu_data.calib_AX);
  EEPROM.writeFloat(4, mpu_data.calib_AY);
  EEPROM.writeFloat(8, mpu_data.calib_AZ);
  EEPROM.writeFloat(12, mpu_data.calib_GX);
  EEPROM.writeFloat(16, mpu_data.calib_GY);
  EEPROM.writeFloat(20, mpu_data.calib_GZ);
  EEPROM.commit();
  Serial.println("Calibration Completed");
  digitalWrite(LED, LOW);
}

void Matrix_transpose(float mat[2][2], float res[2][2])
{
  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 2; j++)
    {
      res[i][j] = mat[j][i];
    }
  }
}
void Matrix_add(float m1[2][2], float m2[2][2], float res[2][2])
{

  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 2; j++)
    {
      res[i][j] = m1[i][j] + m2[i][j];
    }
  }
}
void Matrix_sub(float m1[2][2], float m2[2][2], float res[2][2])
{

  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 2; j++)
    {
      res[i][j] = m1[i][j] - m2[i][j];
    }
  }
}
void Matrix_mul(float m1[2][2], float m2[2][2], float res[2][2])
{

  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 2; j++)
    {
      res[i][j] = 0;
      for (int k = 0; k < 2; k++)
      {
        res[i][j] += m1[i][k] * m2[k][j];
      }
    }
  }
}
void Matrix_copy(float m1[2][2], float m2[2][2])
{
  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 2; j++)
    {
      m1[i][j] = m2[i][j];
    }
  }
}
void predict_state(float attitude, float rate, float state_predicted[2][2])
{
  float A[2][2] = {{1, DT}, {0, 1}};
  float X[2][2] = {{attitude}, {rate}};
  attitude += rate * DT;
  // roll_rate = gyrox;
  // float B[2][2] = {{(DT)}, {1}};
  // float U[2][2] = {{gyrox}};
  // float state_matrix[2][2];
  // float control_matrix[2][2];
  Matrix_mul(A, X, state_predicted);
  // Matrix_mul(B, U, control_matrix);
  // Matrix_add(state_matrix, control_matrix, state_predicted);
}
void Estimate_Roll(float *Roll, float *Roll_rate)
{
 if(DT > 0.1)
  {
    DT = 0.01;
  }
  float state_predicted_Roll[2][2];
  float A[2][2] = {{1, DT}, {0, 1}};
  float process_covar_Roll[2][2];
  float A_transpose[2][2];
  float temp[2][2];
  float temp1[2][2];
  float temp2[2][2];
  MPU_data();
  float AccelX = mpu_data.AX - mpu_data.calib_AX;
  float AccelY = mpu_data.AY - mpu_data.calib_AY;
  float AccelZ = mpu_data.AZ - mpu_data.calib_AZ;
  float GyroX = mpu_data.GX - mpu_data.calib_GX;
  float GyroY = mpu_data.GY - mpu_data.calib_GY;
  float GyroZ = mpu_data.GZ - mpu_data.calib_GZ;
  Accel_Roll = atan2(AccelY, AccelZ) * 180 / M_PI;
  Matrix_transpose(A, A_transpose);
  Matrix_mul(A, init_process_covar, temp);
  Matrix_mul(temp, A_transpose, temp1);
  float Q[2][2] = {{0.001, 0}, {0, 0.01}}; // process noise
  Matrix_add(temp1, Q, process_covar_Roll);     // process covariance
  process_covar_Roll[0][1] = 0;
  process_covar_Roll[1][0] = 0;
  predict_state(state_estimated_Roll[0][0], GyroX, state_predicted_Roll);
  float H[2][2] = {{1, 0}, {0, 0}};
  float H_transpose[2][2];
  Matrix_transpose(H, H_transpose);
  Matrix_mul(H, process_covar_Roll, temp);
  Matrix_mul(temp, H_transpose, temp1);
  float R[2][2] = {{0.01, 0}, {0, 0}}; // measurement noise
  Matrix_add(temp1, R, temp);          // denominator of kalman gain
  Matrix_mul(process_covar_Roll, H_transpose, temp1);
  if (fabs(temp[0][0]) < 1e-6)
  {
    temp[0][0] = 1e-6; // or a slightly larger value
  }
  float kalman_gain[2][2] = {{(temp1[0][0] / temp[0][0])},
                             {(temp1[1][0] / temp[0][0])}};
  float measurement[2][2] = {{Accel_Roll}};
  Matrix_mul(H, state_predicted_Roll, temp);
  Matrix_sub(measurement, temp, temp1);
  Matrix_mul(kalman_gain, temp1, temp);
  Matrix_add(state_predicted_Roll, temp, state_estimated_Roll);
  Matrix_mul(kalman_gain, H, temp1);
  float identity[2][2] = {{1, 0}, {0, 1}};
  Matrix_sub(identity, temp1, temp);
  Matrix_mul(temp, process_covar_Roll, init_process_covar);
  *Roll = state_estimated_Roll[0][0];
  *Roll_rate = state_estimated_Roll[1][0];
}
void Estimate_Pitch(float *Pitch, float *Pitch_rate)
{
  if(DT > 0.1)
  {
    DT = 0.01;
  }
  float state_predicted_Pitch[2][2];
  float A[2][2] = {{1, DT}, {0, 1}};
  float process_covar_Pitch[2][2];
  float A_transpose[2][2];
  float temp[2][2];
  float temp1[2][2];
  float temp2[2][2];
  MPU_data();
  float AccelX = mpu_data.AX - mpu_data.calib_AX;
  float AccelY = mpu_data.AY - mpu_data.calib_AY;
  float AccelZ = mpu_data.AZ - mpu_data.calib_AZ;
  float GyroX = mpu_data.GX - mpu_data.calib_GX;
  float GyroY = mpu_data.GY - mpu_data.calib_GY;
  float GyroZ = mpu_data.GZ - mpu_data.calib_GZ;
  Accel_Pitch = atan2(AccelX, sqrt(AccelY * AccelY + AccelZ * AccelZ)) * 180 / M_PI;
  Matrix_transpose(A, A_transpose);
  Matrix_mul(A, init_process_covar, temp);
  Matrix_mul(temp, A_transpose, temp1);
  float Q[2][2] = {{0.001, 0}, {0, 0.01}}; // process noise
  Matrix_add(temp1, Q, process_covar_Pitch);     // process covariance
  process_covar_Pitch[0][1] = 0;
  process_covar_Pitch[1][0] = 0;
  predict_state(state_estimated_Pitch[0][0], GyroY, state_predicted_Pitch);
  float H[2][2] = {{1, 0}, {0, 0}};
  float H_transpose[2][2];
  Matrix_transpose(H, H_transpose);
  Matrix_mul(H, process_covar_Pitch, temp);
  Matrix_mul(temp, H_transpose, temp1);
  float R[2][2] = {{0.01, 0}, {0, 0}}; // measurement noise
  Matrix_add(temp1, R, temp);          // denominator of kalman gain
  Matrix_mul(process_covar_Pitch, H_transpose, temp1);
  if (fabs(temp[0][0]) < 1e-6)
  {
    temp[0][0] = 1e-6; // or a slightly larger value
  }
  float kalman_gain[2][2] = {{(temp1[0][0] / temp[0][0])},
                             {(temp1[1][0] / temp[0][0])}};
  float measurement[2][2] = {{Accel_Pitch}};
  Matrix_mul(H, state_predicted_Pitch, temp);
  Matrix_sub(measurement, temp, temp1);
  Matrix_mul(kalman_gain, temp1, temp);
  Matrix_add(state_predicted_Pitch, temp, state_estimated_Pitch);
  Matrix_mul(kalman_gain, H, temp1);
  float identity[2][2] = {{1, 0}, {0, 1}};
  Matrix_sub(identity, temp1, temp);
  Matrix_mul(temp, process_covar_Pitch, init_process_covar);
  *Pitch = state_estimated_Pitch[0][0];
  *Pitch_rate = state_estimated_Pitch[1][0];
}




