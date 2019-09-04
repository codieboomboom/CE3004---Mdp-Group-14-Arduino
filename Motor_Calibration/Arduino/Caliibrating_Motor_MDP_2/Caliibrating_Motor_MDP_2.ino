//A Program to help in calibrating the motor's rpm
#include "DualVNH5019MotorShield.h"
DualVNH5019MotorShield md;
#define sampleSize  500
#define sampleFrom  125
#define sampleTo    375

int encoderA = 3; //3 for motor 1, 11 for motor 2
int encoderB = 5; //5 for motor 1, 13 for motor 2
unsigned long timing = 0;
int sample = sampleSize;
int i = -400;

boolean collectData = true;

void setup() {
  // put your setup code here, to run once:
  md.init();
  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);
  Serial.begin(9600);
  
}

void loop() {
  // put your main code here, to run repeatedly:
//  md.setM2Speed(400);
//  timingA = pulseIn(encoderA, HIGH);
//  //timingB = pulseIn(encoderB, HIGH);
//  Serial.println(timingA);
//  delay(4000);
//  md.setM2Speed(0);
while (collectData){
  md.setM1Speed(i);
  if(sample>1){
    noInterrupts();
    timing = pulseIn(encoderB, HIGH, 50000UL);
    interrupts();
    //timingB = pulseIn(encoderA, HIGH);
    if (sample <= sampleTo && sample >= sampleFrom)
      {Serial.println(timing);}
    sample--;
  }
  else{
    if(i<400){
      i+=20;
      sample = sampleSize;
      Serial.print("PWM set: ");
      Serial.println(i);
    }
    else{
      Serial.println("Done!");
      md.setM1Speed(0); 
      collectData = false;
    }
  }
} 
}
