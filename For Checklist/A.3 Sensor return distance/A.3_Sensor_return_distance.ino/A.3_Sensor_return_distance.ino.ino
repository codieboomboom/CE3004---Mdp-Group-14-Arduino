#include "DualVNH5019MotorShield.h"
#include "SharpIR.h"
#include "EnableInterrupt.h"


DualVNH5019MotorShield md;
SharpIR ps1_long(SharpIR:: GP2Y0A02YK0F, A0);
SharpIR ps2_short_Front_Left(SharpIR:: GP2Y0A21YK0F, A1);
SharpIR ps3_short_Front_Right(SharpIR:: GP2Y0A21YK0F, A2);
SharpIR ps4_short_Right(SharpIR:: GP2Y0A21YK0F, A3);


/*
 * This code is meant to complete checklist A.3.
 * PID is not included.
 * Amended from Movement_withSensor_v1.2
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


/*-----Flags-----*/
bool stopMoving_flag = true;


void print_Distance(){
  Serial.println(" ");
  Serial.print("ps2_short_Front_Left: ");
  Serial.print(ps2_short_Front_Left.getDistance());
  Serial.print("  ps3_short_Front_Right: ");
  Serial.println(ps3_short_Front_Right.getDistance());
  Serial.print("  ps1_longt: ");
  Serial.println(ps1_long.getDistance());
}


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
}

void loop ()
{

  while(Serial.available())
  {
    char command;
    command = Serial.read();
    
    if(command == 's'){
      stopMoving_flag = true;
    }
    else if(command == 'w'){
      stopMoving_flag = false;
    }
  }

  if(stopMoving_flag == true){
    print_Distance();
  }
  else{
    front_Calibrate();
  }
  delay(100);
}

void front_Calibrate(){

  Serial.println("Front Calibrating");
  
  while(ps2_short_Front_Left.getDistance() != 10 || ps3_short_Front_Right.getDistance() != 10){
    
    print_Distance();
    
    //No obstacle straight in front
    if(ps2_short_Front_Left.getDistance() > 10 && ps3_short_Front_Right.getDistance() > 10){
      md.setSpeeds(100, 100);
    }
    //Wide obstacle straight in front
    else if(ps2_short_Front_Left.getDistance() < 10 && ps3_short_Front_Right.getDistance() < 10){
      md.setBrakes(400, 400);
    }
    //Incoming obstacle on the left
    else if(ps2_short_Front_Left.getDistance() < 10 && ps3_short_Front_Right.getDistance() > 10){
      md.setBrakes(400, 400);
    }
    //Incoming obstacle on the right
    else if(ps2_short_Front_Left.getDistance() > 10 && ps3_short_Front_Right.getDistance() < 10){
      md.setBrakes(400, 400);
    }
  }
  md.setBrakes(400, 400);
}
