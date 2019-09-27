/*
 * ==================================
 * MDP Group 14 - Sem 1 AY19/20
 * -----------------------------
 * Description: An Arduino script for controlling robot hardwares such as motors, sensors for MDP project.
 * Authors: Anh Tu & Bronson
 * Version: 2.3 (Checklist Oriented)
 * ==================================
 */

#include <EnableInterrupt.h>
#include "DualVNH5019MotorShield.h"
#include <PID_v1.h>

//motor 1 on the right
//motor 2 on the left
int encoder1A = 3;
int encoder1B = 5;
int encoder2A = 11;
int encoder2B = 13;

double circumference = PI * 6;
double distance_cm; //distance in cm that the robot need to move
double speed1, speed2;
double currentTick1, currentTick2, oldTick1, oldTick2;

volatile long tick1 = 0;
volatile long tick2 = 0;
long ticks_moved = 0;

char command[50];

DualVNH5019MotorShield md;
PID PIDControlLeft(&currentTick1, &speed1, &currentTick2, 3, 0, 0.2, DIRECT);
PID PIDControlRight(&currentTick1, &speed1, &currentTick2, 6.5, 3, 0.5, DIRECT);
PID PIDControl(&currentTick1, &speed1, &currentTick2, 7.5 ,3 ,0.1, DIRECT); //15 5 0.1
PID PIDCheckList(&currentTick1, &speed1, &currentTick2, 0 ,0 ,0, DIRECT);
/*
 * =================================================
 * PID Theory:
 * Kp: Proportional term, decide how fast and how strong the system response to a change. System should be in an oscillating state (turn left and right and left)
 * Kd: Differential term, decide how dampen the oscillation will be. Higher = more dampened
 * Ki: How strong the response is if error is accumulated. (hard turn in extreme case)
 * -------------------------------------------------
 * Differential PID with Master Slave configuration:
 * Master: Stable, slower Motor
 * Slave: Unstable, faster motor
 * We feed a speed into the system, then measure the actual speed of both motor. 
 * We then calculate the error in this speed and 
 * calculate the PID adjustment to be feed back so the error is balanced out.
 * Here, we want to control the speed of unstable motor (1) so that it is as close and stable to stable motor (2)'s speed.
 * We feedback to the system as PWM speed (speed1) of the unstable motor 1.
 * ===================================================
 */
void setup() {
  Serial.begin(9600);
  //Serial.println("Connected");
  md.init();
  //Attach interrupts
  pinMode(encoder1A,INPUT);
  enableInterrupt(encoder1A, E1_Pos, RISING);
  pinMode(encoder2A,INPUT);
  enableInterrupt(encoder2A, E2_Pos, RISING);
  currentTick1 = currentTick2 = oldTick1 = oldTick2 = 0;
//  moveForwardCheckList(150);
//  delay(50);
//  rotateLeft(89);
//  delay(50);
//  rotateLeft(89);
//  delay(50);
//  moveForwardCheckList(150);
//  rotateLeft(400);
  
}

void loop() {
  count = 0;
  while(command[count] != '\0'){
    switch(command[count]){
      case 'W':
      case 'w':
    }
  }
}

/*
 * ===================================
 * Motion Control
 * ===================================
 */
 //Method to move straight with easy PID
void moveForward(){

   tick1 = tick2 = 0;
   currentTick1 = currentTick2 = 0;
   oldTick1 = oldTick2 = 0;
   speed1 = speed2 = 220;
   md.setSpeeds(speed1,speed2);
   delay(50);
   oldTick1 = (double)tick1;
   oldTick2 = (double)tick2;
   PIDControl.SetSampleTime(50);
   PIDControl.SetMode(AUTOMATIC);

   while(1){
    currentTick1 = tick1 - oldTick1;
    currentTick2 = tick2 - oldTick2;
    Serial.print(currentTick1);
    Serial.print(" "); Serial.println(currentTick2);
    bool yes = PIDControl.Compute();
    md.setSpeeds(speed1, speed2);
    oldTick2 += currentTick2;
    oldTick1 += currentTick1;
    //oldTick2 = tick2;
    //oldTick1 = tick1;
   }
   
}
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

   while(tick_travelled < target_tick){
    // if not reach destination ticks yet
    currentTick1 = tick1 - oldTick1; //calculate the ticks travelled in this sample interval of 50ms
    currentTick2 = tick2 - oldTick2;
    
    Serial.print(currentTick1); //for debug
    Serial.print(" "); Serial.println(currentTick2);
    
    bool yes = PIDControl.Compute(); //return 1 if PID compute something, else will return 0;
    
    md.setSpeeds(speed1, speed2); //adjusting speed based on computed PID (PID compute new speed1)
    
    oldTick2 += currentTick2; //update ticks
    oldTick1 += currentTick1;
    tick_travelled += currentTick2;
    //oldTick2 = tick2;
    //oldTick1 = tick1;
   }
   //gradual breaking once destination is reached to preven jerking
   for (int i = 0; i <= 400; i+=50){
     md.setBrakes(i-5,i+5);
     delay(2.5); 
   }
//   md.setBrakes(100,100);
//   delay(5);
//   md.setBrakes(200,200);
//   delay(5);
//   md.setBrakes(400,400);
//   delay(5);
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

      Serial.print(currentTick1); //for debug
      Serial.print(" "); Serial.println(currentTick2);
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

      Serial.print(currentTick1); //for debug
      Serial.print(" "); Serial.println(currentTick2);
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

//A Method to clear the checklist going straight for certain distance
void moveForwardCheckList(double distance){
   //Calculate the amount of motor ticks needed to reach the distance
   double target_tick = distanceToTicks(distance+10);
   double tick_travelled = 0;
   Serial.print("Target: ");
   Serial.println(target_tick);
   if(target_tick<0) return;

   // Init values
   tick1 = tick2 = 0; //encoder's ticks (constantly increased when the program is running due to interrupt)
   currentTick1 = currentTick2 = 0; //ticks that we are used to calculate PID. Ticks at the current sampling of PIDController
   oldTick1 = oldTick2 = 0;

   //Speed in rpm for motor 1 and 2
   speed1 = rpmToPWM_1(81.8);
   speed2 = rpmToPWM_2(59.7);
   Serial.println("Speed:");
   Serial.print(speed1);Serial.print(" ");
   Serial.println(speed2);

   //Implementing gradual acceleration to remove jerks
   for (int j = 0; j < speed2; j+=50){
     md.setSpeeds(j-15,j+5);
     delay(5); 
   }

   //Set Final ideal speed and accomodate for the ticks we used in acceleration
   md.setSpeeds(speed1,speed2);
   tick_travelled = (double)tick2;

   PIDCheckList.SetSampleTime(50); //Controller is called every 50ms
   PIDCheckList.SetMode(AUTOMATIC); //Controller is invoked automatically.

   while(tick_travelled < target_tick){
      // if not reach destination ticks yet
      currentTick1 = tick1 - oldTick1; //calculate the ticks travelled in this sample interval of 50ms
      currentTick2 = tick2 - oldTick2;
      
      Serial.print(currentTick1); //for debug
      Serial.print(" "); Serial.println(currentTick2);
      PIDCheckList.Compute();

      oldTick2 += currentTick2; //update ticks
      oldTick1 += currentTick1;
      tick_travelled += currentTick2;
      //oldTick2 = tick2;
      //oldTick1 = tick1;
   }
   
   for (int i = 0; i <= 400; i+=50){
     md.setBrakes(i-5,i+5);
     delay(2.5); 
   }
   PIDCheckList.SetMode(MANUAL);
}



/*
 * ===============================
 * Conversion methods for motors
 * ===============================
 */

double distanceToTicks(double distance){
  return ((0.95*distance) * 562.25)/circumference;
}

double rpmToPWM_1(double RPM){
  double pwmMotor1 = 2.7037*(RPM) - 21.27;
  return pwmMotor1;
}

double rpmToPWM_2(double RPM){
  double pwmMotor2 = 2.6872*RPM + 44.236;
  return pwmMotor2;
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
 * ============================
 * Sensors
 * ============================
 */
