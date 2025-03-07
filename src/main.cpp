//Flight controller
#include <Arduino.h>
#include <ESP32Servo.h>
#define Aileron_stick_IN 36  // Aileron stick input
#define Elevator_stick_IN 39 // Elevator stick input
#define Throttle_stick_IN 34 // Throttle stick input
#define Rudder_stick_IN 35   // Rudder stick input
#define Mode_stick_IN 32     // Mode switch input
#define Manual_Mode 0
#define FBWA_Mode 1
#define FBWB_Mode 2
#define Max_aileron_delfection 20
#define Max_elevator_delfection 20
#define Max_rudder_delfection 20

Servo Aileron_servo;
Servo Elevator_servo;
Servo Throttle_servo;
Servo Rudder_servo;

float Aileron_norm;
float Elevator_norm;
float Throttle_norm;
float Rudder_norm;
int Flight_mode;

// put function declarations here:
// int myFunction(int, int);
void setup()
{
  // put your setup code here, to run once:
  pinMode(Aileron_stick_IN, INPUT);
  pinMode(Elevator_stick_IN, INPUT);
  pinMode(Throttle_stick_IN, INPUT);
  pinMode(Rudder_stick_IN, INPUT);
  pinMode(Mode_stick_IN, INPUT);
  Aileron_servo.attach(33);
  Elevator_servo.attach(25);
  Throttle_servo.attach(26);
  Rudder_servo.attach(27);
  Serial.begin(9600);

  // int result = myFunction(2, 3);
}

void read_PWM()
{
  float Aileron_raw = pulseIn(Aileron_stick_IN, HIGH);
  float Elevator_raw = pulseIn(Elevator_stick_IN, HIGH);
  float Throttle_raw = pulseIn(Throttle_stick_IN, HIGH);
  float Rudder_raw = pulseIn(Rudder_stick_IN, HIGH);
  float Mode_raw = pulseIn(Mode_stick_IN, HIGH);
  // Serial.println("raw values");
  // Serial.println(Aileron_raw);
  // Serialx.println(Elevator_raw);
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
    Flight_mode = FBWB_Mode;
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

void control_outputs()
{
  if (Flight_mode == Manual_Mode)
  {
    float Aileron_PWM = 500 * (Aileron_norm + 1) + 1000;
    float Elevator_PWM = 500 * (Elevator_norm + 1) + 1000;
    float Throttle_PWM = 1000 * (Throttle_norm) + 1000;
    float Rudder_PWM = 500 * (Rudder_norm + 1) + 1000;
  
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
}

void loop()
{
   Serial.println("Normalised values");
   Serial.println(Aileron_norm);
   Serial.println(Elevator_norm);
   Serial.println(Throttle_norm);
   Serial.println(Rudder_norm);
   Serial.println("-----------");
  read_PWM();
  control_outputs();
  // delay(1000);
  //  put your main code here, to run repeatedly:
}

// put function definitions here:
// int myFunction(int x, int y) {
//   return x + y;
// }