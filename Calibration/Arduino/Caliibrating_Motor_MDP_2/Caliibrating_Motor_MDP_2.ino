//A Program to help in calibrating the motor's rpm
#include "DualVNH5019MotorShield.h"
DualVNH5019MotorShield md;
#define sampleSize  500
#define sampleFrom  125
#define sampleTo    375

int encoder1A = 3; //3 for motor 1, 11 for motor 2
int encoder1B = 5; //5 for motor 1, 13 for motor 2
int encoder2A = 11;
int encoder2B = 13;
unsigned long timing = 0;
int sample = sampleSize;
int i = -400;

boolean collectData = true;

void setup() {
  // put your setup code here, to run once:
  md.init();
  pinMode(encoder1A, INPUT);
  pinMode(encoder1B, INPUT);
  pinMode(encoder2A, INPUT);
  pinMode(encoder2B, INPUT);
  Serial.begin(9600);
  Serial.println("PWM set: -400");
  
}

void loop() {
while (collectData){
  md.setM2Speed(i); //change
  if(sample>1){
    noInterrupts();
    timing = pulseIn(encoder2A, HIGH, 10000UL); //change
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
      md.setM2Speed(0); //change
      collectData = false;
    }
  }
} 
}
