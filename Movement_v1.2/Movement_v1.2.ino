#include "DualVNH5019MotorShield.h"
#include "SharpIR.h"
#include "EnableInterrupt.h"
#include "TimerOne.h" 

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
double inverted_M1_speed = 0;
double inverted_M2_speed = 0;

/*-----PID Variables-----*/
double M1_setpoint_ticks = 200;
double M2_setpoint_ticks = 200;
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
int currentdirection = 0;

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

  Timer1.initialize(1000000);
  Timer1.attachInterrupt(PIDController);

}

void loop ()
{
  while(Serial.available())
  {
    char command;
    command = Serial.read();
    
    if(command == 'w'){
      Serial.println("I received w.");
      currentdirection = 1;
      forward();
      flag = true;
    }
    else if(command == 's'){
      Serial.println("I received s.");
      currentdirection = 2;
      backward();
      flag = true;
    }
    else if(command == 'a'){
      Serial.println("I received a.");
      currentdirection = 3;
      left();
      flag = true;
    }
    else if(command == 'd'){
      Serial.println("I received d.");
      currentdirection = 4;
      right();
      flag = true;
    }
    
    //stop operation
    if(command == 'q'){
      Serial.println("I received q.");
      Serial.print("E1_sum_error: ");
      Serial.println(E1_sum_error);
      Serial.print("E2_sum_error: ");
      Serial.println(E2_sum_error);
      md.setBrakes(400, 400);
      flag = false;
    }
  }


  /*if(flag == true && currentdirection != 0){
      PIDController(currentdirection);
      delay(1000);*/
  }
}

void forward(){
  PIDController(1);
}

void backward(){
  PIDController(2);
}

void left(){
  PIDController(3);
}

void right(){
  PIDController(4);
}


/*-----PID-----*/
//https://projects.raspberrypi.org/en/projects/robotPID/5
void PIDController(int directionflag)
{
  Serial.println("PID is running");
  E1_error_ticks = M1_setpoint_ticks - E1_ticks;
  E2_error_ticks = M2_setpoint_ticks - E2_ticks;

  //Calculation
  M1_speed = (E1_error_ticks * (KP * 0.95)) + (E1_prev_error * (KD + 0.01)) + (E1_sum_error * KI);
  M2_speed = (E2_error_ticks * KP) + (E2_prev_error * KD) + (E2_sum_error * KI);
  //Motor 1 usually goes faster than Motor 2.

  //Just in case
  M1_speed = max(min(250, M1_speed), 0);
  M2_speed = max(min(250, M2_speed), 0);

  inverted_M1_speed = -M1_speed;
  inverted_M2_speed = -M2_speed;  
  Serial.print("inverted_M1_speed: ");
  Serial.print(inverted_M1_speed);
  Serial.print("     inverted_M2_speed: ");
  Serial.println(inverted_M2_speed);
  Serial.print("directionflag: ");
  Serial.println(directionflag);
  
  //Motor 1 is right motor, Motor 2 is left motor.
  //forward
  if (directionflag == 1){
    md.setSpeeds(M1_speed, M2_speed);
    Serial.println("md.setSpeeds(M1_speed, M2_speed);");
  }
  //backward
  else if(directionflag == 2){
    md.setSpeeds(inverted_M1_speed, inverted_M2_speed);
    Serial.println("md.setSpeeds(inverted_M1_speed, inverted_M2_speed);");
  }
  //left
  else if(directionflag == 3){
    md.setSpeeds(M1_speed, inverted_M2_speed);
    Serial.println("md.setSpeeds(M1_speed, inverted_M2_speed);");
  }
  //right
  else if(directionflag == 4){
    md.setSpeeds(inverted_M1_speed, M2_speed);
    Serial.println("md.setSpeeds(inverted_M1_speed, M2_speed);");
  }
  

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
