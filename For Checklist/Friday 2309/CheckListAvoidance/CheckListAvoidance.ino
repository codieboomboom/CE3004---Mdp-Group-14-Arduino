#include "DualVNH5019MotorShield.h"
#include "SharpIR.h"
#include <EnableInterrupt.h>
#include <PID_v1.h>

DualVNH5019MotorShield md;
SharpIR ps1_short_Front_Left(SharpIR:: GP2Y0A21YK0F, A0);
SharpIR ps2_short_Back_Left(SharpIR:: GP2Y0A21YK0F, A1);
SharpIR ps3_short_Front_Middle(SharpIR:: GP2Y0A21YK0F, A2);
SharpIR ps4_short_Front_Right(SharpIR:: GP2Y0A21YK0F, A3);
SharpIR ps5_long(SharpIR:: GP2Y0A02YK0F, A4);
SharpIR ps6_short_Back_Right(SharpIR:: GP2Y0A21YK0F, A5);

//Motor 1 on the right
#define encoder1A_INPUT 3
#define encoder1B_INPUT 5

//Motor 2 on the left
#define encoder2A_INPUT 11
#define encoder2B_INPUT 13

double currentTick1, currentTick2, oldTick1, oldTick2;
bool rightWall_flag;

double circumference = PI * 6;
double distance_cm; //distance in cm that the robot need to move
double speed1, speed2;

volatile long tick1 = 0;
volatile long tick2 = 0;
long ticks_moved = 0;

PID PIDControl(&currentTick1, &speed1, &currentTick2, 7.5 ,3 ,0.1, DIRECT);
PID PIDControlLeft(&currentTick1, &speed1, &currentTick2, 3, 0, 0.2, DIRECT);
PID PIDControlRight(&currentTick1, &speed1, &currentTick2, 6.5, 3, 0.5, DIRECT);


void setup() {
  Serial.begin(9600);
  md.init();

  //Motor 1
  pinMode(encoder1A_INPUT, INPUT);
  pinMode(encoder1B_INPUT, INPUT);

  //Motor 2
  pinMode(encoder2A_INPUT, INPUT);
  pinMode(encoder2B_INPUT, INPUT);

  //Interrupts
  enableInterrupt(encoder1A_INPUT, E1_Pos, RISING);
  enableInterrupt(encoder2A_INPUT, E2_Pos, RISING);

  currentTick1 = currentTick2 = oldTick1 = oldTick2 = 0;
  rightWall_flag = false;
}

void loop() {
  moveForward(ps3_short_Front_Middle.getDistance());
  delay(1000);
}

/* ==========================
 * Interrupt Service Routines
 * ==========================
 */
void E1_Pos(){
  tick1++;
}

void E2_Pos(){
  tick2++;
}


/*
 * ===================================
 * Motion Control
 * ===================================
 */

//Method to move straight with a stipiulated distance.
void moveForward(double distance_cm){
  
  //Calculate the amount of motor ticks needed to reach the distance
   double target_tick = distanceToTicks(distance_cm);
   double tick_travelled = 0;
   
   if(target_tick<0) return;

   // Init values
   tick1 = tick2 = 0; //encoder's ticks (constantly increased when the program is running due to interrupt)
   currentTick1 = currentTick2 = 0; //ticks that we are used to calculate PID. Ticks at the current sampling of PIDController
   oldTick1 = oldTick2 = 0; //All ticks accumulated until the current PIDController is called.
   speed1 = 227;
   speed2 = 230; //Speed in PWM of the system.

   //Implementing gradual acceleration to remove jerks
   for (int j = 0; j < speed2; j+=50){
     md.setSpeeds(j-15,j+5);
     delay(5); 
   }

   //Set Final ideal speed and accomodate for the ticks we used in acceleration
   md.setSpeeds(speed1,speed2);
   tick_travelled = (double)tick2;

   //All ticks accumulated until now is reflected in the encoder's ticks
   oldTick1 = (double)tick1;
   oldTick2 = (double)tick2;

   //PID stuffs
   PIDControl.SetSampleTime(50); //Controller is called every 50ms
   PIDControl.SetMode(AUTOMATIC); //Controller is invoked automatically.

   while(tick_travelled < target_tick || (ps1_short_Front_Left.getDistance() > 12 && ps4_short_Front_Right.getDistance() > 12 && ps3_short_Front_Middle.getDistance() > 10)){
    // if not reach destination ticks yet or is clear to move forward
    currentTick1 = tick1 - oldTick1; //calculate the ticks travelled in this sample interval of 50ms
    currentTick2 = tick2 - oldTick2;
    
    //Serial.print(currentTick1); //for debug
    //Serial.print(" "); Serial.println(currentTick2);
    
    bool yes = PIDControl.Compute(); //return 1 if PID compute something, else will return 0;

    md.setSpeeds(speed1, speed2); //adjusting speed based on computed PID (PID compute new speed1)
    oldTick2 += currentTick2; //update ticks
    oldTick1 += currentTick1;
    tick_travelled += currentTick2;
      

    //Wide obstacle straight in front
    if(ps1_short_Front_Left.getDistance() < 12 && ps4_short_Front_Right.getDistance() < 12 && ps3_short_Front_Middle.getDistance() < 10){
       //md.setSpeeds(-100, -100);
       wall_Check();
       //print_Distance();
       break;
    }
    //Incoming obstacle on the left
    else if(ps1_short_Front_Left.getDistance() < 12 && ps4_short_Front_Right.getDistance() > 12){
       md.setBrakes(400, 400);
       rotateRight(90);
       Serial.println("Left corner blocked - Turning right");
       break;
    }
    //Incoming obstacle on the right
    else if(ps1_short_Front_Left.getDistance() > 12 && ps4_short_Front_Right.getDistance() < 12){
       md.setBrakes(400, 400);
       delay(100);
       rotateLeft(90);
       Serial.println("Right corner blocked - Turning left");
       delay(100);
       break;
    }

   }
   //gradual breaking once destination is reached to preven jerking
   for (int i = 0; i <= 400; i+=50){
     md.setBrakes(i-5,i+5);
     delay(2.5); 
   }

}

//A method to rotate robot to the left by a degree. Using 360 degree as a base line
void rotateLeft(double degree)
{
  double target_tick; 
  //0-720: 4.5039x - 36.745
  //720-1080: 0.025x^2 - 35.686x +15980
  if (degree <= 720){
    target_tick = 4.5039*degree - 23.745;
  }
  else if(degree>900){
    target_tick = 4.3905*degree +126.04;
  }
  else{
    target_tick = 4.7389*degree -190;
  }
  //360 = 1565 ticks| 180 = 783 ticks 90 = 391 |720 = 1565*2 + 40 ticks |1080 = 1565*4 + 190
  //With PID 1: 360 = 1570 |720 = 3140
  //90: 381 ; 180:781; 270: 1173; 360:1565; 540:2386; 720:3222; 900:4075; 1080:6545

  //target_tick = 4850;
  double tick_travelled = 0;
  if (target_tick<0) return;
  
  // Init values
  tick1 = tick2 = 0; //encoder's ticks (constantly increased when the program is running due to interrupt)
  currentTick1 = currentTick2 = 0; //ticks that we are used to calculate PID. Ticks at the current sampling of PIDController
  oldTick1 = oldTick2 = 0;
  //speed1 = rpmToPWM_1(70);
  //speed2 = rpmToPWM_2(-70);
  speed1 = 210;
  speed2 = -210;

  
  md.setSpeeds(speed1,speed2);
  tick_travelled = (double)tick2;

  PIDControlLeft.SetSampleTime(50); //Controller is called every 50ms
  PIDControlLeft.SetMode(AUTOMATIC); //Controller is invoked automatically.

  while(tick_travelled < target_tick){
      // if not reach destination ticks yet
      currentTick1 = tick1 - oldTick1; //calculate the ticks travelled in this sample interval of 50ms
      currentTick2 = tick2 - oldTick2;

      //Serial.print(currentTick1); //for debug
      //Serial.print(" "); Serial.println(currentTick2);
      PIDControlLeft.Compute();
      oldTick2 += currentTick2; //update ticks
      oldTick1 += currentTick1;
      tick_travelled += currentTick2;
      //oldTick2 = tick2;
      //oldTick1 = tick1;
  }

     for (int i = 0; i <= 400; i+=100){
     md.setBrakes(i-5,i+5);
     delay(2.5); 
   }
   PIDControlLeft.SetMode(MANUAL);
  
}


//A method to rotate robot to the right by a degree. Using 360 degree as a base line
void rotateRight(double degree)
{
  double target_tick; 
//  if (degree <= 720){
//    target_tick = 4.5039*degree - 23.745;
//  }
//  else if(degree>900){
//    target_tick = 4.3905*degree +126.04;
//  }
//  else{
//    target_tick = 4.7389*degree -190;
//  }

  target_tick = 4.3215*degree - 27.165;
  double tick_travelled = 0;
  if (target_tick<0) return;
  
  // Init values
  tick1 = tick2 = 0; //encoder's ticks (constantly increased when the program is running due to interrupt)
  currentTick1 = currentTick2 = 0; //ticks that we are used to calculate PID. Ticks at the current sampling of PIDController
  oldTick1 = oldTick2 = 0;
  //speed1 = rpmToPWM_1(70);
  //speed2 = rpmToPWM_2(-70);
  speed1 = -210;
  speed2 = 210;

  
  md.setSpeeds(speed1,speed2);
  tick_travelled = (double)tick2;

  PIDControlRight.SetSampleTime(50); //Controller is called every 50ms
  PIDControlRight.SetMode(AUTOMATIC); //Controller is invoked automatically.

  while(tick_travelled < target_tick){
      // if not reach destination ticks yet
      currentTick1 = tick1 - oldTick1; //calculate the ticks travelled in this sample interval of 50ms
      currentTick2 = tick2 - oldTick2;

      //Serial.print(currentTick1); //for debug
      //Serial.print(" "); Serial.println(currentTick2);
      PIDControlRight.Compute();
      oldTick2 += currentTick2; //update ticks
      oldTick1 += currentTick1;
      tick_travelled += currentTick2;
      //oldTick2 = tick2;
      //oldTick1 = tick1;
  }

     for (int i = 0; i <= 400; i+=100){
     md.setBrakes(i-5,i+5);
     delay(2.5); 
   }
   PIDControlRight.SetMode(MANUAL); //turn off PID
  
}


void wall_Check(){
  
  Serial.println("Wall Check");
  Serial.println(ps6_short_Back_Right.getDistance());
  
  //Right corner
  if(rightWall_flag == false){
    if(ps6_short_Back_Right.getDistance() <= 12){
      rightWall_flag = true;
      rotateLeft(90);
      Serial.println("Turning left");
    }
    else{ //(ps6_short_Back_Right.getDistance() > 12)
      rotateRight(90);
      Serial.println("Turning right");
    }
  }
  //Left corner
  else{ //(rightWall_flag == true)
    if(ps6_short_Back_Right.getDistance() <= 12){
      rotateLeft(90);
      Serial.println("Turning left");
    }
    else{ //(ps6_short_Back_Right.getDistance() > 12)
      rightWall_flag = false;
      rotateRight(90);
      Serial.println("Turning right");
    }
  }
}


void print_Distance(){
  Serial.println(" ");
  Serial.print("ps1_short_Front_Left: ");
  Serial.print(ps1_short_Front_Left.getDistance());
  Serial.print("  ps4_short_Front_Right: ");
  Serial.println(ps4_short_Front_Right.getDistance());
  Serial.print("  ps3_short_Front_Middle: ");
  Serial.println(ps3_short_Front_Middle.getDistance());
}

double distanceToTicks(double distance){
  return ((0.95*distance) * 562.25)/circumference;
}
