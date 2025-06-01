// Flight controller
#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <IMU.h>
#define Aileron_stick_IN 36  // Aileron stick input
#define Elevator_stick_IN 39 // Elevator stick input
#define Throttle_stick_IN 34 // Throttle stick input
#define Rudder_stick_IN 35   // Rudder stick input
#define Mode_stick_IN 32     // Mode switch input


#define Aileron_out_ch 33  //
#define Elevator_out_ch 25 //
#define Throttle_out_ch 26 // 
#define Rudder_out_ch 27   // 

#define Manual_Mode 0
#define FBWA_Mode 1
#define FBWB_Mode 2
#define Max_del_a 20
// #define Max_elevator_delfection 20
// #define Max_rudder_delfection 20
#define Max_Roll_Angle 30
#define Max_Pitch_Angle 30
#define D2R 0.0174532925
#define R2D 57.2957795131

//Gains
#define k_roll_tconst 2.5
#define k_roll_ff 0.01
#define k_roll_kp 0.09
#define k_roll_ki 0.001 
#define roll_ki_max 0.34
#define k_pitch_tconst  0.5
#define k_pitch_ff 0.1
#define k_pitch_kp 0.1
#define k_pitch_ki 0.1
#define pitch_ki_max 0.34

Servo Aileron_servo;
Servo Elevator_servo;
Servo Throttle_servo;
Servo Rudder_servo;


float Roll = 0.0, Roll_rate = 0.0;
float Pitch, Pitch_rate;
float Aileron_norm;
float Elevator_norm;
float Throttle_norm;
float Rudder_norm;
int   Flight_mode;
float Aileron_PWM = 0;
float Elevator_PWM = 0;
float Throttle_PWM = 0;
float Rudder_PWM =0 ;

// put function declarations here:
// int myFunction(int, int);

void Aileron_rise();
void Aileron_fall();
void Elevator_rise();
void Elevator_fall();
void Throttle_rise();
void Throttle_fall();
void Rudder_rise();
void Rudder_fall();
void mode_rise();
void mode_fall();
void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  EEPROM.begin(EEPROM_Size);
  pinMode(LED, OUTPUT);
  pinMode(Push_Button, INPUT);
  pinMode(Push_Button, INPUT_PULLUP);
  Wire.begin();
  configure_MPU();
  // calibrate_MPU(2000);// need to develop interrupt based calibration
  Previous_Time = millis() / 1000.0;
  pinMode(Aileron_stick_IN, INPUT);
  pinMode(Elevator_stick_IN, INPUT);
  pinMode(Throttle_stick_IN, INPUT);
  pinMode(Rudder_stick_IN, INPUT);
  pinMode(Mode_stick_IN, INPUT);
  attachInterrupt(Aileron_stick_IN, Aileron_rise, RISING);
  attachInterrupt(Elevator_stick_IN, Elevator_rise, RISING);
  attachInterrupt(Throttle_stick_IN, Throttle_rise, RISING);
  attachInterrupt(Rudder_stick_IN, Rudder_rise, RISING);
  attachInterrupt(Mode_stick_IN, mode_rise, RISING);
  

  Aileron_servo.attach(Aileron_out_ch);
  Elevator_servo.attach(Elevator_out_ch);
  Throttle_servo.attach(Throttle_out_ch);
  Rudder_servo.attach(Rudder_out_ch);

  // int result = myFunction(2, 3);
}
volatile int Aileron_PWM_start = 0;
volatile int Aileron_PWM_end = 0;
volatile int Elevator_PWM_start = 0;
volatile int Elevator_PWM_end = 0;
volatile int Throttle_PWM_start = 0;
volatile int Throttle_PWM_end = 0;
volatile int Rudder_PWM_start = 0;
volatile int Rudder_PWM_end = 0;
volatile int Mode_PWM_start = 0;
volatile int Mode_PWM_end = 0;
void Aileron_rise()
{
  Aileron_PWM_start = micros();
  attachInterrupt(Aileron_stick_IN, Aileron_fall, FALLING);
}
void Aileron_fall()
{
  Aileron_PWM_end = micros();
  attachInterrupt(Aileron_stick_IN, Aileron_rise, RISING);
}
void Elevator_rise()
{
  Elevator_PWM_start = micros();
  attachInterrupt(Elevator_stick_IN, Elevator_fall, FALLING);
}
void Elevator_fall()
{
  Elevator_PWM_end = micros();
  attachInterrupt(Elevator_stick_IN, Elevator_rise, RISING);
}
void Throttle_rise()
{
  Throttle_PWM_start = micros();
  attachInterrupt(Throttle_stick_IN, Throttle_fall, FALLING);
}
void Throttle_fall()
{
  Throttle_PWM_end = micros();
  attachInterrupt(Throttle_stick_IN, Throttle_rise, RISING);
}
void Rudder_rise()
{
  Rudder_PWM_start = micros();
  attachInterrupt(Rudder_stick_IN, Rudder_fall, FALLING);
}

void Rudder_fall()
{
  Rudder_PWM_end = micros();
  attachInterrupt(Rudder_stick_IN, Rudder_rise, RISING);
}
void mode_rise()
{
  Mode_PWM_start = micros();
  attachInterrupt(Mode_stick_IN, mode_fall, FALLING);
}
void mode_fall()
{
  Mode_PWM_end = micros();
  attachInterrupt(Mode_stick_IN, mode_rise, RISING);
}

void read_PWM()
{
  int Aileron_raw = Aileron_PWM_end - Aileron_PWM_start;
  int Elevator_raw = Elevator_PWM_end - Elevator_PWM_start;
  int Throttle_raw = Throttle_PWM_end - Throttle_PWM_start;
  int Rudder_raw = Rudder_PWM_end - Rudder_PWM_start;
  int Mode_raw = Mode_PWM_end - Mode_PWM_start;
  // Serial.println(Aileron_raw);
  // Serial.println(Elevator_raw);
  // Serial.println(Throttle_raw);
  // Serial.println(Rudder_raw);
  // Serial.println(Mode_raw);
  //float Aileron_raw = pulseIn(Aileron_stick_IN, HIGH);
  // float Elevator_raw = pulseIn(Elevator_stick_IN, HIGH);
  // float Throttle_raw = pulseIn(Throttle_stick_IN, HIGH);
  // float Rudder_raw = pulseIn(Rudder_stick_IN, HIGH);
 //  float Mode_raw = pulseIn(Mode_stick_IN, HIGH);
  // Serial.println("raw values");
  // Serial.println(Aileron_raw);
  // Serial.println(Elevator_raw);
  // Serial.println(Throttle_raw);
  // Serial.println(Rudder_raw);
  // Serial.println(Mode_raw);
  // Serial.println("-----------");
  // Read flight mode

  if (Mode_raw < 1100)
  {
    Flight_mode = Manual_Mode;
  }

  if ((Mode_raw >= 1200) && (Mode_raw < 1700))
  {
    Flight_mode = FBWA_Mode;
  }
  if (Mode_raw >= 1700)
  {
    Flight_mode = FBWA_Mode;
  }

  // Normalise aileron stick input

  float Aileron_norm_trim = 1490;
  float Aileron_norm_min = 994;
  float Aileron_norm_max = 2000;

  if (Aileron_raw <= (Aileron_norm_trim - 10))

  {
    Aileron_norm = (((0 + 1) / ((Aileron_norm_trim - 10) - Aileron_norm_min)) * (Aileron_raw - Aileron_norm_min)) - 1;
  }

  if (((Aileron_raw > (Aileron_norm_trim - 10)) && (Aileron_raw < (Aileron_norm_trim + 10))) || Aileron_raw == 0)

  {
    Aileron_norm = 0;
  }

  if (Aileron_raw >= (Aileron_norm_trim + 10))
  {
    Aileron_norm = ((1 - 0) / (Aileron_norm_max - (Aileron_norm_trim + 10))) * (Aileron_raw - (Aileron_norm_trim + 10));
  }

  // Normalise Elevator stick input
  float Elevator_norm_trim = 1490;
  float Elevator_norm_min = 994;
  float Elevator_norm_max = 2000;
  if (Elevator_raw <= (Elevator_norm_trim - 10))
  {
    Elevator_norm = (((0 + 1) / ((Elevator_norm_trim - 10) - Elevator_norm_min)) * (Elevator_raw - Elevator_norm_min)) - 1;
  }
  if (((Elevator_raw > (Elevator_norm_trim - 10)) && (Elevator_raw < (Elevator_norm_trim + 10))) || Elevator_raw == 0)
  {
    Elevator_norm = 0;
  }

  if (Elevator_raw >= (Elevator_norm_trim + 10))
  {

    Elevator_norm = ((1 - 0) / (Elevator_norm_max - (Elevator_norm_trim + 10))) * (Elevator_raw - (Elevator_norm_trim + 10));
  }
  // Normalise Throttle stick input

  float Throttle_norm_trim = 1030;
  float Throttle_norm_max = 2000;
  if (Throttle_raw < (Throttle_norm_trim + 10))
  {

    Throttle_norm = 0;
  }
  if (Throttle_raw >= (Throttle_norm_trim + 10))
  {
    Throttle_norm = ((1 - 0) / (Throttle_norm_max - (Throttle_norm_trim + 10))) * (Throttle_raw - (Throttle_norm_trim + 10));
  }

  // Normalise Rudder stick input
  float Rudder_norm_trim = 1490;
  float Rudder_norm_min = 994;
  float Rudder_norm_max = 2000;
  if (Rudder_raw <= (Rudder_norm_trim - 10))
  {
    Rudder_norm = (((0 + 1) / ((Rudder_norm_trim - 10) - Rudder_norm_min)) * (Rudder_raw - Rudder_norm_min)) - 1;
  }
  if (((Rudder_raw > (Rudder_norm_trim - 10)) && (Rudder_raw < (Rudder_norm_trim + 10))) || Rudder_raw == 0)
  {

    Rudder_norm = 0;
  }
  if (Rudder_raw >= (Rudder_norm_trim + 10))
  {

    Rudder_norm = ((1 - 0) / (Rudder_norm_max - (Rudder_norm_trim + 10))) * (Rudder_raw - (Rudder_norm_trim + 10));
  }
  
  
}
float bound(float value , float min, float max)
{
  if (value <= min)
  {
    return min;
  }
  else if (value >= max)
  {
    return max;
  }
  else
  {
    return value;
  }
}
 float roll_i_sum = 0;
 float roll_i_out = 0;
float PID(float error)
{
  
  if(abs(roll_i_sum) < roll_ki_max)
  {
    roll_i_sum = roll_i_sum + error * DT;
    roll_i_sum = bound(roll_i_sum, -roll_ki_max, roll_ki_max);
  }
  else
  {
    roll_i_sum = roll_i_sum;
  }
  float PID_out = error * k_roll_kp + roll_i_sum * k_roll_ki;
  return PID_out;
}
float Roll_innerLoop(float Roll_Angle_cmd)
{
  Estimate_Roll(&Roll, &Roll_rate);
  Serial.println(Roll);
  float Angle_err = (Roll_Angle_cmd * D2R) - (Roll * D2R);
  float Roll_rate_cmd = k_roll_tconst * Angle_err;
  float rate_err = Roll_rate_cmd - (Roll_rate * D2R);
  float Roll_ff_out = rate_err * k_roll_ff;
  float roll_pid_out = PID(rate_err) + Roll_ff_out;
  float del_a = bound(roll_pid_out, -(Max_del_a*D2R), (Max_del_a*D2R));
 
  return del_a;
}
float fmap(float value, float min, float max)
{
  return ((value - min) * 2.0 / (max - min)) - 1.0;
}
void control_outputs()
{
  if (Flight_mode == Manual_Mode)
  {
     Aileron_PWM = 500 * (Aileron_norm + 1) + 1000;
     Elevator_PWM = 500 * (Elevator_norm + 1) + 1000;
     Throttle_PWM = 1000 * (Throttle_norm) + 1000;
     Rudder_PWM = 500 * (Rudder_norm + 1) + 1000;

    // Serial.println("Servo PWMs");
    // Serial.println(Aileron_PWM);
    // Serial.println(Elevator_PWM);
    // Serial.println(Throttle_PWM);
    // Serial.println(Rudder_PWM);
    // Serial.println("-----------");
    Aileron_servo.writeMicroseconds(Aileron_PWM);
    Elevator_servo.writeMicroseconds(Elevator_PWM);
    Throttle_servo.writeMicroseconds(Throttle_PWM);
    Rudder_servo.writeMicroseconds(Rudder_PWM);
  }
  
  if(Flight_mode == FBWA_Mode)
  {
    float Roll_angle_cmd = Aileron_norm * Max_Roll_Angle;
    float del_a =  Roll_innerLoop(Roll_angle_cmd);
    float del_a_norm = fmap(del_a, -Max_del_a*D2R, Max_del_a*D2R);
    //Serial.println(del_a_norm);
     Aileron_PWM = 500 * (del_a_norm + 1) + 1000;
     //Serial.println(del_a);
     Aileron_servo.writeMicroseconds(Aileron_PWM);
  }
}

void loop()
{

  Current_Time = millis() / 1000.0;
  DT = Current_Time - Previous_Time;
  Previous_Time = Current_Time;
  //i2c_scanner();
   if (digitalRead(Push_Button) == LOW)
  {
    calibrate_MPU(3000);
  }
  read_PWM();
  control_outputs();
  // Serial.println("Normalised values");
  // Serial.println(Aileron_norm);
  // Serial.println(Elevator_norm);
  // Serial.println(Throttle_norm);
  // Serial.println(Rudder_norm);
  // Serial.println("-----------");
  // Estimate_Roll(&Roll, &Roll_rate);
  // Serial.println(Roll);
  // Estimate_Pitch(&Pitch, &Pitch_rate);
  // Serial.print(":");
  // Serial.println(Pitch);
  delay(5);
  //  put your main code here, to run repeatedly:
}

// put function definitions here:
// int myFunction(int x, int y) {
//   return x + y;
// }