//A program to test robot after calibrating motors
#include "DualVNH5019MotorShield.h"
#include "EnableInterrupt.h"

DualVNH5019MotorShield md;
//For PID Purpose:
#define KP1 1.5  //multiplier 1.5
#define KI1 0.7 //0.6
#define KD1 KI1/4  //
#define KP2 1.3 //multiplier 1.6
#define KI2 0.6 //0.2
#define KD2 KI2/4 //0.05
#define INTERVAL 40   //constant interval, minimum 16
#define SIZE 40

int encoder1A = 3; //3 for motor 1, 11 for motor 2
int encoder1B = 5; //5 for motor 1, 13 for motor 2
int encoder2A = 11;
int encoder2B = 13;

double rpm1, rpm2; //RPM of motor 1 and 2
double pwm1, pwm2; //PWM value for setting speed of motor 1 and 2 
double previous1, previous2;
double P, I, D;

double diff = 0; prev_diff = 0; //error
unsigned long time1, time2;
volatile unsigned long tick1 = 0, tick2 = 0, oldtick1=0, oldtick2 = 0;
volatile unsigned long current1, current2;
int step_count = 0;


void setup() {
  //Reset Handler Routine - to be executed first
  Serial.begin(115200); //init serial communication
  Serial.println("Connected to Arduino"); //For debugging purpose
  setupMotor();
  //init value of pwm on both sides to 0
  pwm1 = 0;
  pwm2 = 0;
  //init value of previous calculated Rpm on both sides to current rpm
  previousRpm1 = rpm1;
  previousRpm2 = rpm2;
  //init startpoint for sampling
  time1 = millis();
  time2 = millis();
  //init current ticks number
  currentTick1 = tick1;
  currentTick2 = tick2;
  

}

void loop() {
  // put your main code here, to run repeatedly:

}

/*
 * Method to setup motor shield and polling for interrupts
 */
void setupMotor(){
  md.init();
  pinMode(encoder1A, INPUT);
  pinMode(encoder2B, INPUT);
  enableInterrupt(encoder1A, motorISR1, CHANGE);
  enableInterrupt(econder1B, motorISR2, CHANGE);
}

/*
 * ISRs to increase ticks number for either motor encoders when there is a change in edge.
 * Half period of a square wave
 */
void motorISR1(){
  tick1++;
}

void motorISR2(){
  tick2++;
}

/*
 * A method to make robot go straight
 */
void goStraight(){
  
}
