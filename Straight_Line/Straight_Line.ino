#include "DualVNH5019MotorShield.h"
#include "SharpIR.h"
#include "EnableInterrupt.h"

DualVNH5019MotorShield md;
SharpIR sensor(SharpIR:: GP2Y0A02YK0F, A0);

/*
 * This code is meant to complete checklist A.4
 * Amnded from Movement_v1.2.3
 * Remove existing commands and program the robot to move forward when turned on.
 * Findings:
 * 1. Feedback increases infinitely - fixed using Limit
 * 2. As the rate of PID is being called decreases, Anthu's equation becomes less accurate.
 * 3. Sometimes the robot doesn't move forward when turned on - connection between the portable charger is loose, reconnect it.
 */

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
double M1_speed_pwm = 0;
double M2_speed_pwm = 0;
double inverted_M1_speed_pwm = 0;
double inverted_M2_speed_pwm = 0;

/*-----PID Variables-----*/
double E1_rpm = 0;
double E2_rpm = 0;
double M1_setpoint_rpm = 40;
double M2_setpoint_rpm = 40;
double E1_error_rpm = 0;
double E2_error_rpm = 0;
double E1_prev_error_rpm = 0;
double E2_prev_error_rpm = 0;
double E1_sum_error_rpm = 0;
double E2_sum_error_rpm = 0;
double M1_speed_rpm = 0;
double M2_speed_rpm = 0;

double KP1 = 1.525;
double KD1 = 0.67;
double KI1 = 1.37;
double KP2 = 1.475;
double KD2 = 0.38;
double KI2 = 0.8;

/*-----Flags-----*/
bool justStarted_flag = true;
int currentdirection = 0;
unsigned long startTime = 0;

void setup()
{
  Serial.begin(9600);
  md.init();


  //Motor 1
  digitalWrite(enable_M1, HIGH);
  
  pinMode(output_1A, OUTPUT);
  pinMode(output_1B, OUTPUT);
  
  pinMode(encoder1A_INPUT, INPUT);
  pinMode(encoder1B_INPUT, INPUT);
  pulseIn(encoder1A_INPUT, HIGH);



  //Motor 2
  digitalWrite(enable_M2, HIGH);
  
  pinMode(output_2A, OUTPUT);
  pinMode(output_2B, OUTPUT);
  
  pinMode(encoder2A_INPUT, INPUT);
  pinMode(encoder2B_INPUT, INPUT);
  pulseIn(encoder2A_INPUT, HIGH);

  //Interrupts
  enableInterrupt(encoder1A_INPUT, E1_ticks_increment, RISING);
  enableInterrupt(encoder2A_INPUT, E2_ticks_increment, RISING);
}

void loop ()
{  
  PIDController(1);
  delay(1000);
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
  //Debug log to check if PID is running.
  Serial.println("");
  Serial.println("PID is running.");

  //Debug log to see ticks generated.
  print_ticks();

  //Get Period and Debug log to see period.
  double period = (double)(millis()-startTime)/1000;
  //Serial.print("Sampling interval: ");
  //Serial.println(period);

  /*-----Convert Ticks to RPM-----*/
  E1_rpm = (((double)E1_ticks/562.25)/period)*60;
  E2_rpm = (((double)E2_ticks/562.25)/period)*60;

  //Debug log to see RPM converted from Ticks from Encoders.
  //print_converted_RPM();


  
  /*-----PID Calculation using RPM-----*/
  E1_error_rpm = M1_setpoint_rpm - E1_rpm;
  E2_error_rpm = M2_setpoint_rpm - E2_rpm;
  //print_errors();
  
  M1_speed_rpm += (E1_error_rpm * KP1) + (E1_prev_error_rpm * KD1);// + (E1_sum_error_rpm * KI1);
  M2_speed_rpm += (E2_error_rpm * KP2) + (E2_prev_error_rpm * KD2);// + (E2_sum_error_rpm * KI2);
  //Motor 1 usually goes faster than Motor 2.

  /*-----Limit-----*/
  M1_speed_rpm = max(min(60, M1_speed_rpm), 0);
  M2_speed_rpm = max(min(60, M2_speed_rpm), 0);
  
  //Debug log to see RPM calculated from PID.
  print_feedback_RPM();

  
  /*-----Convert RPM to PWM-----*/
  M1_speed_pwm = rpmToPWM_1(M1_speed_rpm);
  M2_speed_pwm = rpmToPWM_2(M2_speed_rpm);
  inverted_M1_speed_pwm = -M1_speed_pwm;
  inverted_M2_speed_pwm = -M2_speed_pwm;

  //Debug log to see PWM converted from RPM calulated from PID.
  print_converted_PWM();

  
  /*-----Setting Motor Speed using PWM-----*/
  //Motor 1 is right motor, Motor 2 is left motor.
  //forward
  if (directionflag == 1){
    md.setSpeeds(M1_speed_pwm, M2_speed_pwm);
    //Serial.println("direction: FORWARD");
  }
  //backward
  else if(directionflag == 2){
    md.setSpeeds(inverted_M1_speed_pwm, inverted_M2_speed_pwm);
    //Serial.println("direction: BACKWARD");
  }
  //left
  else if(directionflag == 3){
    md.setSpeeds(M1_speed_pwm, inverted_M2_speed_pwm);
    //Serial.println("direction: LEFT");
  }
  //right
  else if(directionflag == 4){
    md.setSpeeds(inverted_M1_speed_pwm, M2_speed_pwm);
    //Serial.println("direction: RIGHT");
  }
  

  E1_ticks_moved += E1_ticks;
  E2_ticks_moved += E2_ticks;
      
  
  /*-----Reset-----*/ 
  E1_ticks = 0;
  E2_ticks = 0;
  startTime = millis();

  /*-----Update-----*/
  E1_prev_error_rpm = E1_error_rpm;
  E2_prev_error_rpm = E2_error_rpm;

  E1_sum_error_rpm += E1_error_rpm;
  E2_sum_error_rpm += E2_error_rpm;
}


void E1_ticks_increment()
{
  E1_ticks ++;
}

void E2_ticks_increment()
{
  E2_ticks ++;
}

double rpmToPWM_1(double RPM){
  double pwmMotor1 = 2.7037*RPM + 39.27;
  return pwmMotor1;
}

double rpmToPWM_2(double RPM){
  double pwmMotor2 = 2.7037*RPM + 34.236;
  return pwmMotor2;
}



/*-----Serial Prints-----*/
void print_converted_RPM(){
  //Serial.println("");
  //Serial.print("E1_rpm: ");
  Serial.print(E1_rpm);
  Serial.print(" ");
  //Serial.print("   E2_rpm: ");
  Serial.println(E2_rpm);
}

void print_ticks(){
  Serial.println("");
  Serial.print("E1_ticks: ");
  Serial.print(E1_ticks);
  Serial.print("     E2_ticks: ");
  Serial.println(E2_ticks);
}

void print_feedback_RPM(){
  Serial.println("");
  Serial.print("M1_speed_rpm: ");
  Serial.print(M1_speed_rpm);
  Serial.print("   M2_speed_rpm: ");
  Serial.println(M2_speed_rpm);
}

void print_converted_PWM(){
  Serial.println("");
  Serial.print("M1_speed_pwm: ");
  Serial.print(M1_speed_pwm);
  Serial.print("   M2_speed_pwm: ");
  Serial.println(M2_speed_pwm);
}

void print_errors(){
  Serial.println("");
  Serial.print("E1_error_rpm: ");
  Serial.print(E1_error_rpm);
  Serial.print("   E2_error_rpm: ");
  Serial.println(E2_error_rpm);
}
