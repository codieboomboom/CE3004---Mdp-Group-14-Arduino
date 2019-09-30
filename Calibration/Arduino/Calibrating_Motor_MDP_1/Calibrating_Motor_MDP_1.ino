//A Program to help in calibrating the motor's rpm
#include "DualVNH5019MotorShield.h"
#include "EnableInterrupt.h"

DualVNH5019MotorShield md;
int encoderA = 11; //3 for motor 1, 11 for motor 2
int encoderB = 13; //5 for motor 1, 13 for motor 2
volatile unsigned int tick = 0; //init tick number to be zero
unsigned long StartTime = 0;
unsigned long EndTime = 0;
unsigned long TimeWidth = 0;


void setup() {
  md.init();
  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);
  enableInterrupt(encoderA, doEncoder, RISING);
  //enableInterrupt(encoderB, doEncoder, CHANGE);
  Serial.begin(9600);
  Serial.println("Start the program now...");
  md.setM2Speed(400);
  StartTime = micros();
  Serial.println(StartTime);
}

void loop() {
  // put your main code here, to run repeatedly:
}

void doEncoder(){
  /* if either encoderA or encoderB change from High-Low or Low-High,
   *  increase the tick count by 1.
   *  When ticks hit 2249, stop the program and calculate the time taken
   */
  if (tick == 0)
  {
    tick++;
  }
  else if (tick < 512)
  {
    tick++;
  }
  else {
    EndTime = micros();
    md.setM2Speed(0);
    Serial.println(EndTime);
    TimeWidth = EndTime - StartTime;
    Serial.println(tick); 
    Serial.println(TimeWidth);
    tick = 0;    
  }
}
