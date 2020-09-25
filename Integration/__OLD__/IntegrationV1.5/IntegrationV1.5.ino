/*
 * Version: 1.15
 * 
 * Description:
 * This code is part of the MDP 2019 done by group 14. 
 * This code is written for integration testing of Arduino Robot Hardware to Rpi via serial communication. 
 * 
 * Author: 
 * Anh Tu Do U1721947F
 * 
 * Features:
 * 1. Serial buffer and serial communication to interact with RPI. Data to be sent back to RPI are pending on RPI, Android and Algo team.
 *  1.1. Template for serial communication done, can tolerate a string of 50 characters
 * 2. Calibration using front sensors when reaching a wall // object
 * 3. Accurate & deterministic straight line motion, rotation and simple object avoidance.
 *  3.1. Moving straight 2x2 grid in 1-6 blocks without much deviation. Discrete motion only (not continous yet)
 *  3.2. Rotate Left and Right exactly by 90 degree.
 *  3.3. Motor Characteristic calibrated.
 * 4. Attempt of having 2 operating mode for robot FAST and NORMAL.
 * 5. Sensor able to return distance in grid/block away from obstacle.
 *  5.1. Using sensor with mean filter.
 *  
 * Notes & Issues:
 * - Implementing of Median filter and comparison for best performance
 * - Experimenting with different sample size to ensure most accurate sensor reading but good time complexity
 * 
 * 
 */


/*
 * ===================================
 * Packages 
 * DualVNH5019MotorShield - Library to interface with Pololu motorshield to control DC motors
 * SharpIR - Library to interface with Sharp IR sensors
 * EnableInterrupt - Library to allows interrupts control on any digital pin. Note: Arduino Uno can only configure D0 and D1 as interrupt 
 * pin by AttachInterrupt() without this library.
 * PID_v1 - Library to easy and quick integration of PID Controller
 * ===================================
 */

#include "DualVNH5019MotorShield.h"
#include "SharpIR.h"
#include <EnableInterrupt.h>
#include <PID_v1.h>
#include <ArduinoSort.h>


/*
 * ==============================
 * Variables declaration
 * ==============================
 */

//Define Encoder Pins
int encoder1A = 3;
int encoder1B = 5;
int encoder2A = 11;
int encoder2B = 13;

double distance_cm; //distance in cm that the robot need to move.
double speed1, speed2; //speed (in PWM) to feed to the motorshield.

//ticks parameters for PID
volatile long tick1 = 0;
volatile long tick2 = 0;
long ticks_moved = 0;
double currentTick1, currentTick2, oldTick1, oldTick2;

//Serial buffer for communication
char command[50];
int count = 0;

//Operating states
bool FAST;
bool DEBUG = true;
bool CALIBRATE = true;

//For sensors
#define SAMPLE 40
#define front_center A0
#define front left A1
#define front right A2
#define right_front A3
#define right_back A4
#define long_left A5
volatile double sensor_reading[SAMPLE]; //for median filtering

//constructors
DualVNH5019MotorShield md;
//SharpIR front_center(SharpIR:: GP2Y0A21YK0F, A0);
//SharpIR front_left(SharpIR:: GP2Y0A21YK0F, A1);
//SharpIR front_right(SharpIR:: GP2Y0A21YK0F, A2);
//SharpIR top_right(SharpIR:: GP2Y0A21YK0F, A3);
//SharpIR bottom_right(SharpIR:: GP2Y0A21YK0F, A4);
//SharpIR long_left(SharpIR:: GP2Y0A02YK0F, A5);

//Refer to end of program for explanation on PID
PID PIDControlStraight(&currentTick1, &speed1, &currentTick2, 1.3 ,0.1 ,0, DIRECT);
PID PIDControlLeft(&currentTick1, &speed1, &currentTick2, 0, 0, 0, DIRECT);
PID PIDControlRight(&currentTick1, &speed1, &currentTick2, 0, 0, 0, DIRECT);

/*
 * ==============================
 * Main Program
 * ==============================
 */
void setup() {
  Serial.begin(9600);
  if (DEBUG){
    Serial.println("Connected");
  }
  
  md.init();
  
  //Attach interrupts to counts ticks
  pinMode(encoder1A,INPUT);
  enableInterrupt(encoder1A, E1_Pos, RISING);
  pinMode(encoder2A,INPUT);
  enableInterrupt(encoder2A, E2_Pos, RISING);

  //init values
  currentTick1 = currentTick2 = oldTick1 = oldTick2 = 0;
  //rotate_left(1);

  for (int j = 0; j < 10; j--){
    sensor_calibrate();
    delay(50); //try if delay time affect reading.
  }
}

void loop() {
  get_command();
  //Debug to see all commands collected.
//  if(DEBUG){
//    print_all_commands();
//  }
  count=0;
  while (command[count] != '\0'){
    //while not end of string
    switch(command[count]){
      case 'W':
      case 'w':
        //move the robot forward with stipulated distance unit (blocks)
        switch(command[count+1]){
          //check how many blocks to move
          case '0':
          case '1':
          case '2':
          case '3':
          case '4':
          case '5':
          case '6':
          case '7':
          case '8':
          case '9':
            if(DEBUG){
              Serial.print("Moving Forward by ");
              Serial.print(command[count+1]);
              Serial.println(" units"); 
            }
            move_forward(int(command[count+1])-48); //moving forward by amount stipulcated.
            //offset -48 is to convert ASCII to blocks distance (cuz 0 = 48 for ascii decimal and so on so forth)
            break;
            
          default: 
            if(DEBUG) {
                Serial.println("S's ERROR INVALID COMMAND: "); 
                Serial.println(command[count]); 
             }
             break;
             
            }
        count++;
        break;
        
      case 'A':
      case 'a':
        //rotate robot to the left 90 degrees
        if (DEBUG){Serial.println("Rotating Left by 90 degrees");}
        rotate_left(90);
        break;
        
      case 'D':
      case 'd':
        //rotate robot to the right 90 degrees
        if (DEBUG){Serial.println("Rotating Right by 90 degrees");}
        rotate_right(90);
        break;
      
      default: //by default means error command
        if(DEBUG) {
          Serial.print("ERROR INVALID COMMAND: "); 
          Serial.println(command[count]); 
        }
        break;
    }
    count++;
  }
  command[0] = '\0';
}

/*
 * =======================================================
 * Motion Controls
 * Methods to move the robot straight, left, right, etc...
 * =======================================================
 */

 //A method to rotate robot to the right by a degree. Using 360 degree as a base line
void rotate_right(double degree)
{
  double target_tick = 0; 
  target_tick =  0.2319*degree + 6.4492;
  double tick_travelled = 0;
  if (target_tick<0) return;
  
  // Init values
  tick1 = tick2 = 0; //encoder's ticks (constantly increased when the program is running due to interrupt)
  currentTick1 = currentTick2 = 0; //ticks that we are used to calculate PID. Ticks at the current sampling of PIDController
  oldTick1 = oldTick2 = 0;
  speed1 = rpm_to_speed_1(-70);
  speed2 = rpm_to_speed_2(68.5);

  
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
     md.setBrakes(i,i);
     delay(2.5); 
   }
   PIDControlLeft.SetMode(MANUAL);
   delay(100);
}

//A method to rotate robot to the left by a degree. Using 360 degree as a base line
void rotate_left(double degree)
{
  double target_tick = 0;
  target_tick = 0.2267*degree + 4.6766;
  double tick_travelled = 0;
  if (target_tick<0) return;
  
  // Init values
  tick1 = tick2 = 0; //encoder's ticks (constantly increased when the program is running due to interrupt)
  currentTick1 = currentTick2 = 0; //ticks that we are used to calculate PID. Ticks at the current sampling of PIDController
  oldTick1 = oldTick2 = 0;
  speed1 = rpm_to_speed_1(70);
  speed2 = rpm_to_speed_2(-69);
  //speed1 = -210;
  //speed2 = 210;

  
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
   delay(100);
}

//A method to move robot forward by distance/unit of square
void move_forward(double distance){
  //at 6.10v to 6.20v
   double target_tick = 0; 
   //calibrate by set a random value and measure actual distance
   //295 = 1 block, 600 = 2 blocks,910 = 3 blocks,1217 = 4 blocks, 1524 = 5 blocks, 1840 = 6 blocks  
   Serial.println(distance);
   target_tick = 308.69*distance - 19.067;
   double tick_travelled = 0;
   Serial.print("Target: ");
   Serial.println(target_tick);
   if(target_tick<0) return;

   // Init values
   tick1 = tick2 = 0; //encoder's ticks (constantly increased when the program is running due to interrupt)
   currentTick1 = currentTick2 = 0; //ticks that we are used to calculate PID. Ticks at the current sampling of PIDController
   oldTick1 = oldTick2 = 0;

   //Speed in rpm for motor 1 and 2
   speed1 = rpm_to_speed_1(70.75);
   speed2 = rpm_to_speed_2(70.5);
   Serial.println("Speed:");
   Serial.print(speed1);Serial.print(" ");
   Serial.println(speed2);

   //Implementing gradual acceleration to remove jerks
   for (int j = 0; j < speed2; j+=50){
     md.setSpeeds(j+5,j-2.5);
     delay(5); 
   }

   //Set Final ideal speed and accomodate for the ticks we used in acceleration
   md.setSpeeds(speed1,speed2);
   tick_travelled = (double)tick2;

   PIDControlStraight.SetSampleTime(50); //Controller is called every 50ms
   PIDControlStraight.SetMode(AUTOMATIC); //Controller is invoked automatically.

   while(tick_travelled < target_tick){
      // if not reach destination ticks yet
      currentTick1 = tick1 - oldTick1; //calculate the ticks travelled in this sample interval of 50ms
      currentTick2 = tick2 - oldTick2;
      
      Serial.print(currentTick1); //for debug
      Serial.print(" "); Serial.println(currentTick2);
      PIDControlStraight.Compute();

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
   PIDControlStraight.SetMode(MANUAL);
   delay(100);
}

/*
 * ==============================================================
 * Sensors
 * Methods dealing with data acquisition and processing of sensor
 * ==============================================================
 */

 //Methods to return blocks unit from obstacles ++++++++++++
//int distance_obstacle_front_left(){
//  double sum = 0;
//  double distance_cm;
//  double distance_block;
//
//  for (int i = SAMPLE; i > 0; i--){
//    sum += front_left.getDistance();
//  }
//  distance_cm = sum/SAMPLE;
//  
//  return distance_block;
// }
//
//int distance_obstacle_front_right(){
//  double sum = 0;
//  double distance_cm;
//  double distance_block;
//
//  for (int i = SAMPLE; i > 0; i--){
//    sum += front_right.getDistance();
//  }
//  distance_cm = sum/SAMPLE;
//
//  return distance_block;
// }
//
//int distance_obstacle_front_center(){
//  double sum = 0;
//  double distance_cm;
//  double distance_block;
//
//  for (int i = SAMPLE; i > 0; i--){
//    sum += front_center.getDistance();
//  }
//  distance_cm = sum/SAMPLE;
// }
//
//int distance_obstacle_right_front(){
//  double sum = 0;
//  double distance_cm;
//  double distance_block;
//
//  for (int i = SAMPLE; i > 0; i--){
//    sum += top_right.getDistance();
//  }
//  distance_cm = sum/SAMPLE;
// }
//
//int distance_obstacle_right_back(){
//  double sum = 0;
//  double distance_cm;
//  double distance_block;
//
//  for (int i = SAMPLE; i > 0; i--){
//    sum += bottom_right.getDistance();
//  }
//  distance_cm = sum/SAMPLE;
// }
//
//int distance_obstacle_left_long(){
//  double sum = 0;
//  double distance_cm;
//  double distance_block;
//
//  for (int i = SAMPLE; i > 0; i--){
//    sum += long_left.getDistance();
//  }
//  distance_cm = sum/SAMPLE;
// }

//method to aid in calibrating sensors to obtain characteristic of sensor
void sensor_calibrate() {
for(int i=SAMPLE; i>0; i--){
    sensor_reading[i] = analogRead(front_center); //change sensor to be read
  }
sortArray(sensor_reading,SAMPLE);
Serial.print("Analog Reading: "); Serial.println(sensor_reading[SAMPLE/2]);

}
//+++++++++++++++++++++++++++++++++++++++++

//Methods for detect object in front of sensors, for clearance purpose ++++++++++++++++++++++++++++

//+++++++++++++++++++++++++++++++++++++++++
/*
 * ==========================================================================
 * Calibration
 * Methods to align wall, calibrate by front sensors, for turning at corner
 * ==========================================================================
 */

//Method for right wall alignment
void right_wall_calibrate(){
  if(DEBUG) Serial.println("Right Wall calibrating now!");
  
  int i = 0;
  volatile double distance_right_front;
  volatile double distance_right_back;
  
  while (i<80) {
      distance_right_front = short_distance(analogRead(right_front));
      distance_right_back = (short_distance(analogRead(right_back)-0.2));
      if(distance_right_front>33) distance_right_front = 33;
      if(distance_right_back>33) distance_right_back = 33;
      if(distance_right_front<0) distance_right_front = 0;
      if(distance_right_back<0) distance_right_back = 0;
      i++;
      delay(10);  
      if(distance_right_back>distance_right_front){
//        Serial.println("Turn clock");
        if((distance_right_back-distance_right_front)>0.05){
//          Serial.print("LEFT");
          md.setSpeeds(rpm_to_speed_1(70),rpm_to_speed_2(-70));
          delay((distance_right_back-distance_right_front)*7);
        }
      }
      else if(distance_right_front>distance_right_back){
//        Serial.println("Turn anti-clock");
        if((distance_right_front-distance_right_back)>0.05){
          md.setSpeeds(rpm_to_speed_1(-70),rpm_to_speed_2(70));
          delay((distance_right_front-distance_right_back)*7);
        }
    }
    if(DEBUG){
      Serial.print(distance_right_front);
      Serial.print("              |             ");
      Serial.println(distance_right_back);
//      Serial.print(ir_valL1);
//      Serial.print("              |             ");
//      Serial.println(ir_valL2);
  }
      md.setBrakes(400,400);
  }
//  if(!CALIBRATE){
//      if(distance_right_front > 11 && distance_right_front != 33){
//        rotate_left_degree(90);
////        Serial.println((distance_right_front-10)/10);
//        go_straight((distance_right_front-10)/10);
//        delay(100);
//        rotate_right_degree(180);
//        md.setBrakes(400,400);
//        delay(20);
//        calibrate();
//        return;
//      }
//      if(distance_right_front < 9 && distance_right_front != 33){
//        rotate_right_degree(90);
//        go_straight((9.5-distance_right_front)/10);
//        delay(100);
//        md.setBrakes(400,400);
//        delay(20);
//        calibrate();
//        return;
//      }
//      rotate_right_degree(90);
//      calibrate();
//      return;
//  }
}

//method to turn and realign when robot at corner or face obstacle
//NOTE: BY ALGO, ROBOT WILL TURN LEFT FROM ORGINAL DIRECTION. IF OBJECT/WALL IS INFRONT OF ROBOT, IT TURNS LEFT AND GO STRAIGHT TO AVOID OBSTACLES
void right_recalibrate(){
  
}

//Method to calibrate using front sensors
void front_calibrate(){
  
}


/*
 * ==================================
 * Interrupt Service Routine
 * For counting ticks
 * ================================== 
 */
void E1_Pos(){
  tick1++;
}

void E2_Pos(){
  tick2++;
}

/*
 * ======================================================
 * Conversion
 * Methods to convert stuffs
 * ======================================================
 */

double rpm_to_speed_1(double RPM){
  if (RPM>0)
    return 2.8598*RPM + 51.582;
  else if (RPM == 0)
    return 0;
  else
    return -2.9117*(-1)*RPM - 45.197;
}

double rpm_to_speed_2(double RPM){
  if (RPM>0)
    return 2.7845*RPM + 53.789;
  else if (RPM == 0)
    return 0;
  else
    return -2.8109*(-1)*RPM - 54.221;
}

//convert analog reading to distance (CM)
double short_distance(int reading){
  return reading;
}

/*
 * ======================================================
 * Communication
 * Methods to help in communication with RPI
 * ======================================================
 */


//method to get command string into the buffer
void get_command(){
    int i = 0;
    while(Serial.available()>0){
       command[i] = Serial.read();
       i++;
       delay(2); //essential delay cause of serial being too slow
    }
    command[i] = '\0';

    //Debug print command
    if(DEBUG && command[0]!='\0'){
        Serial.print("COMMAND :");
        Serial.println(command);
    }
}

//method to print all characters of string received (for debug)
void print_all_commands(){
  int i = 0;
  Serial.println("Msg Received: ");
  while (command[i] != '\0'){
    Serial.print(command[i]);
    i++;
  }
  Serial.print("EndOfLine");
  Serial.println();
}

/*
 * ========================
 * Others
 * ========================
 *
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
