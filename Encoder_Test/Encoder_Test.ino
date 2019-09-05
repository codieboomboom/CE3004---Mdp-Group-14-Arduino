#include "DualVNH5019MotorShield.h"
#include "SharpIR.h"
#include "EnableInterrupt.h"

DualVNH5019MotorShield md;
SharpIR sensor(SharpIR:: GP2Y0A02YK0F, A0);

//Motor 1
#define encoder1A_INPUT 3
#define encoder1B_INPUT 5
#define output_1A 2
#define output_1B 4
#define enable_M1 6
#define pwm_M1_INPUT 9

//Motor 2
#define encoder2A_INPUT 11
#define encoder2B_INPUT 13
#define output_2A 7
#define output_2B 8
#define enable_M2 12
#define pwm_M2_INPUT 10

/*-----Ticks Variable-----*/
volatile long E1_ticks = 0;
volatile long E2_ticks = 0;

/*-----Check Ticks Variables-----*/
double E1_ticks_moved = 0;
double E2_ticks_moved = 0;
double ticks_to_move = 0;

/*-----Motor Speed Variables-----*/
double M1_speed = 0;
double M2_speed = 0;

/*-----PID Variables-----*/
double setpoint_ticks = 0;
double E1_error_ticks = 0;
double E2_error_ticks = 0;
double E1_prev_error = 0;
double E2_prev_error = 0;
double E1_sum_error = 0;
double E2_sum_error = 0;
double KP = 0.2;
double KD = 0.01;
double KI = 0.005;

bool flag = false;

void setup()
{
  Serial.begin(9600);
  md.init();

//Motor 1
  digitalWrite(enable_M1, HIGH);

  //digitalWrite(pwm_M1_INPUT, HIGH);
  //pinMode(pwm_M1_INPUT, INPUT);
  
  pinMode(output_1A, OUTPUT);
  pinMode(output_1B, OUTPUT);
  
  pinMode(encoder1A_INPUT, INPUT);
  pinMode(encoder1B_INPUT, INPUT);
  pulseIn(encoder1A_INPUT, HIGH);



  //Motor 2
  digitalWrite(enable_M2, HIGH);

  //digitalWrite(pwm_M2_INPUT, HIGH);
  //pinMode(pwm_M2_INPUT, INPUT);
  
  pinMode(output_2A, OUTPUT);
  pinMode(output_2B, OUTPUT);
  
  pinMode(encoder2A_INPUT, INPUT);
  pinMode(encoder2B_INPUT, INPUT);
  pulseIn(encoder2A_INPUT, HIGH);
  pulseIn(encoder2B_INPUT, HIGH);

  
  md.setSpeeds(50, 50);
  enableInterrupt(encoder1A_INPUT, E1_ticks_increment, RISING);
  enableInterrupt(encoder2A_INPUT, E2_ticks_increment, RISING);
  //attachInterrupt(digitalPinToInterrupt(encoder2B_INPUT), E2_ticks_increment, RISING);

}

void loop() {
  
   double val = analogRead(encoder2A_INPUT);
   //Serial.println(val);

   //Serial.println(E1_ticks);
   //Serial.println(E2_ticks);
   //delay(100);
}


void E1_ticks_increment()
{
  Serial.println("E1++");
  E1_ticks ++;
}

void E2_ticks_increment()
{
  Serial.println("E2++");
  E2_ticks ++;
}
