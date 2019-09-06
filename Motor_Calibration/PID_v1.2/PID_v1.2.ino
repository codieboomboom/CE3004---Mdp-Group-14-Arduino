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
double M1_setpoint_ticks = 0;
double M2_setpoint_ticks = 0;
double E1_error_ticks = 0;
double E2_error_ticks = 0;
double E1_prev_error = 0;
double E2_prev_error = 0;
double E1_sum_error = 19251; //16919;
double E2_sum_error = 19591; //17198;
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

  md.setSpeeds(200, 200);
  enableInterrupt(encoder1A_INPUT, E1_ticks_increment, RISING);
  enableInterrupt(encoder2A_INPUT, E2_ticks_increment, RISING);

}

void loop ()
{
  
  while(Serial.available())
  {
    char command;
    command = Serial.read();
    if(command == 'w')  
    {
      Serial.println("I received w.");
      moveforward();
      flag = true;
    }
    if(command == 'a')  
    {
      Serial.println("I received a.");
      Serial.print("E1_sum_error: ");
      Serial.println(E1_sum_error);
      Serial.print("E2_sum_error: ");
      Serial.println(E2_sum_error);
      md.setBrakes(400, 400);
      flag = false;
    }
  }


  if(flag == true){
      PIDController();
      delay(1000);
  }
}

void moveforward()
{
  M1_setpoint_ticks = 200;
  M2_setpoint_ticks = 200;

  //Probably redundant
  ticks_to_move = 1000;
  if(E1_ticks_moved > ticks_to_move || E2_ticks_moved > ticks_to_move)
  {
    Serial.println("Braking");
    md.setBrakes(400, 400);
    delay(100);
  }
  
  PIDController();
}


/*-----PID-----*/
//https://projects.raspberrypi.org/en/projects/robotPID/5
void PIDController()
{
  Serial.println("PID is running");
  E1_error_ticks = M1_setpoint_ticks - E1_ticks;
  E2_error_ticks = M2_setpoint_ticks - E2_ticks;
  
  M1_speed = (E1_error_ticks * (KP * 0.95)) + (E1_prev_error * (KD + 0.01)) + (E1_sum_error * KI);
  M2_speed = (E2_error_ticks * KP) + (E2_prev_error * KD) + (E2_sum_error * KI);

  //Just in case
  M1_speed = max(min(250, M1_speed), 0);
  M2_speed = max(min(250, M2_speed), 0);

  md.setSpeeds(M1_speed, M2_speed);

  E1_ticks_moved += E1_ticks;
  E2_ticks_moved += E2_ticks;

  Serial.print("E1_ticks: ");
  Serial.print(E1_ticks);
  Serial.print("     E2_ticks: ");
  Serial.println(E2_ticks);
      
  Serial.print("M1_speed: ");
  Serial.print(M1_speed);
  Serial.print("     M2_speed: ");
  Serial.println(M2_speed);
      
  //Reset 
  E1_ticks = 0;
  E2_ticks = 0;

  E1_prev_error = E1_error_ticks;
  E2_prev_error = E2_error_ticks;

  E1_sum_error += E1_error_ticks;
  E2_sum_error += E2_error_ticks;
}


void E1_ticks_increment()
{
  E1_ticks ++;
}

void E2_ticks_increment()
{
  E2_ticks ++;
}
