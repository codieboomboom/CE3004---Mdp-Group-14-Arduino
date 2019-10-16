/*
 * Version: 4.0
 * 
 * Description:
 * This code is part of the MDP 2019 done by group 14. 
 * This code is written for integration testing of Arduino Robot Hardware to Rpi via serial communication. 
 * 
 * Author: 
 * Anh Tu Do U1721947F
 * 
 * Features: Releases 4.0
 * Motion Tuning:
 * 1. Going straight: Add in PID for (fastest path, lowering sampling rate
 * 2. Turning Left: lower sampling rate and accurate tuning at exploration speed
 * 3. Turning Right: lower sampling rate and accurate tuning at exploration speed
 * 4. Front calibration: Add in fix for tricky corner case
 * 5. Right calibration: Add in fix for tricky corner case
 * 6. 1-9 ccalibration for fastest path
 * To-Dos:
 * 3. Calibrate turning speed for fastest path at about 90 RPM, tgt with PID and lower sampling rate
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
bool FASTEST_PATH = false;
bool DEBUG = false;
bool CALIBRATE = true;
int delayFastestPath = 20;
//For sensors meian filter
#define SAMPLE 50

volatile double sensor_reading[SAMPLE]; //for median filtering

//constructors
DualVNH5019MotorShield md;
SharpIR front_center(SharpIR:: GP2Y0A21YK0F, A0);
SharpIR front_left(SharpIR:: GP2Y0A21YK0F, A1);
SharpIR front_right(SharpIR:: GP2Y0A21YK0F, A2);
SharpIR right_front(SharpIR:: GP2Y0A21YK0F, A3);
SharpIR right_back(SharpIR:: GP2Y0A21YK0F, A4);
SharpIR long_left(SharpIR:: GP2Y0A02YK0F, A5);

//Refer to end of program for explanation on PID
PID PIDControlStraight(&currentTick1, &speed1, &currentTick2, 3, 0, 0, DIRECT);
PID PIDControlLeft(&currentTick1, &speed1, &currentTick2, 3, 0, 0, DIRECT);
PID PIDControlRight(&currentTick1, &speed1, &currentTick2, 3, 0, 0, DIRECT);

/*
 * ==============================
 * Main Program
 * ==============================
 */
void setup() {
  Serial.begin(9600);
  Serial.setTimeout(50);
  if (DEBUG){
    Serial.println("Connected");
    if(!FASTEST_PATH)
      Serial.println("Exploration mode!");
  }
  
  md.init();
  
  //Attach interrupts to counts ticks
  pinMode(encoder1A,INPUT);
  enableInterrupt(encoder1A, E1_Pos, RISING);
  pinMode(encoder2A,INPUT);
  enableInterrupt(encoder2A, E2_Pos, RISING);

  //init values
  currentTick1 = currentTick2 = oldTick1 = oldTick2 = 0;

//s
//  
//while (true){
//  int front, left, right,right_front_1, right_back_1;
//  for(int i=SAMPLE; i>0; i--){
//      sensor_reading[i] = right_front.getDistance(true);
//      delay(1);
//    }
//  sortArray(sensor_reading,SAMPLE);
//  right_front_1 = sensor_reading[SAMPLE/2]; 
//
//    for(int i=SAMPLE; i>0; i--){
//      sensor_reading[i] = right_back.getDistance(true);
//      delay(1);
//    }
//
//  sortArray(sensor_reading,SAMPLE);
//  right_back_1 = sensor_reading[SAMPLE/2]; 
//
//    for(int i=SAMPLE; i>0; i--){
//      sensor_reading[i] = long_left.getDistance(true);
//      delay(1);
//    }
//
//  sortArray(sensor_reading,SAMPLE);
//  left = sensor_reading[SAMPLE/2]; 
//  Serial.print(right_front_1);Serial.print(" "); Serial.print(right_back_1); Serial.print(" "); Serial.println(left);
//  }
//move_forward(8);
//rotate_left(90);
//move_forward(8);
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
            delay(3);
            move_forward(int(command[count+1])-48); //moving forward by amount stipulcated.
//            for (int i = 0; i< command[count+1]-48; i++){
//              move_forward(1);
//           }
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
        delay(3);
        rotate_left(90);
        Serial.println("MC");
        break;
        
      case 'D':
      case 'd':
        //rotate robot to the right 90 degrees
        if (DEBUG){Serial.println("Rotating Right by 90 degrees");}
        delay(3);
        rotate_right(90);
        Serial.println("MC");
        break;

      case 'H':
      case 'h':
        //calibrate to right wall hug
        if (DEBUG){Serial.println("Right Wall Calibration");}
        delay(3);
        right_wall_calibrate();
        Serial.println("CC");
        break;

      case 'F':
      case 'f':
        //calibrate front
        if (DEBUG){Serial.println("Front Calibrating");}
        delay(3);
        front_calibrate();
        Serial.println("CC");
        break;

      case 'S':
      case 's':
        read_all_sensors(10);
        break;

      case 'E':
      case 'e':
        if (DEBUG){
          Serial.println("Ending and Realigning");
        }
        Serial.println("EC");
        break;

      case 'Z':
      case 'z':
        //enable flag for fastest path
        FASTEST_PATH = true;
        if(DEBUG)
          Serial.println("Get Ready Boiz");
        delay(3);
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
  //target_tick =4.3589*degree - 32.142;
  target_tick = 380;

  if (FASTEST_PATH){
    target_tick = 384;
  }
  //0.2319*degree + 6.4492;
  double tick_travelled = 0;
  if (target_tick<0) return;
  
  // Init values
  tick1 = tick2 = 0; //encoder's ticks (constantly increased when the program is running due to interrupt)
  currentTick1 = currentTick2 = 0; //ticks that we are used to calculate PID. Ticks at the current sampling of PIDController
  oldTick1 = oldTick2 = 0;
  speed1 = rpm_to_speed_1(-70); 
  speed2 = rpm_to_speed_2(69);

  
  md.setSpeeds(speed1,speed2);
  tick_travelled = (double)tick2;

  
  PIDControlRight.SetSampleTime(25); //Controller is called every 25ms
  if (FASTEST_PATH){
    PIDControlRight.SetTunings(4,0, 0.01);
    PIDControlRight.SetSampleTime(15); // less aggressive
  }
  PIDControlRight.SetMode(AUTOMATIC); //Controller is invoked automatically.

  Serial.println("PID controller right: ");
  PIDdebug(PIDControlRight);
  
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

     for (int i = 0; i <= 400; i+=200){
     md.setBrakes(i,i);
     delay(1); 
   }
   PIDControlRight.SetMode(MANUAL);
   
   Serial.println("PID controller right OFF: ");
   PIDdebug(PIDControlRight);
   
   delay(5);
   if(FASTEST_PATH)
      delay(delayFastestPath);
}

//A method to rotate robot to the left by a degree. Using 360 degree as a base line
void rotate_left(double degree)
{
  double target_tick = 0;
  target_tick = 395;

  if(FASTEST_PATH){
    target_tick = 390;
  }
  //target_tick = 4.1533*degree; 
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

  PIDControlLeft.SetSampleTime(25); //Controller is called every 50ms
  if (FASTEST_PATH){
    PIDControlLeft.SetTunings(5,0, 0.01);
    PIDControlLeft.SetSampleTime(15);
  }
  PIDControlLeft.SetMode(AUTOMATIC); //Controller is invoked automatically.

  Serial.println("PID controller left: ");
  PIDdebug(PIDControlLeft);
  

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

     for (int i = 0; i <= 400; i+=200){
     md.setBrakes(i,i);
     delay(1); 
   }
   PIDControlLeft.SetMode(MANUAL); //turn off PID
   Serial.println("PID controller left OFF: ");
   PIDdebug(PIDControlLeft);
   delay(5);
   if(FASTEST_PATH)
      delay(delayFastestPath);
}

//A method to move robot forward by distance/unit of square
void move_forward(int distance){
  //at 6.10v to 6.20v
   double rpm1, rpm2;
   double target_tick = 0; 
   //Serial.println(distance); //debug
   if(FASTEST_PATH){
    switch(distance){
    case 1:
      target_tick = 296;
      break;
    case 2:
      target_tick = 604;
      break;
    case 3:
      target_tick = 926;
      break;
    case 4:
      target_tick = 1237;
      break;
    case 5:
      target_tick = 1547;
      break;
    case 6:
      target_tick = 1860;
      break;
    case 7:
      target_tick = 2170;
      break;
    case 8:
      target_tick = 2487;
      break;
    case 9:
      target_tick = 2810;
      break;
    }
   }
      
   else
      target_tick = 291 ;
   double tick_travelled = 0;
   
//   if(DEBUG){
//   Serial.print("Target: ");
//   Serial.println(target_tick);
//   }

   if(target_tick<0) return;

   // Init values
   tick1 = tick2 = 0; //encoder's ticks (constantly increased when the program is running due to interrupt)
   currentTick1 = currentTick2 = 0; //ticks that we are used to calculate PID. Ticks at the current sampling of PIDController
   oldTick1 = oldTick2 = 0;

   //Speed in rpm for motor 1 and 2
   if (FASTEST_PATH)
   {
    rpm1 = 99;
    rpm2 = 100;
   }
   else{
    rpm1 = 82.5;
    rpm2 = 83;
   }
   speed1 = rpm_to_speed_1(rpm1); //70.75 //74.9  100
   speed2 = rpm_to_speed_2(rpm2); //70.5 //74.5 99.5
   
//   Serial.println("Speed:");//for debug
//   Serial.print(speed1);Serial.print(" ");
//   Serial.println(speed2);

   //Implementing gradual acceleration to remove jerks
   for (int j = 0; j < speed2; j+=50){
     md.setSpeeds(j,j-2.5);
     delay(5); 
   }

   //Set Final ideal speed and accomodate for the ticks we used in acceleration
   md.setSpeeds(speed1,speed2);
   tick_travelled = (double)tick2;
   PIDControlStraight.SetSampleTime(25); //Controller is called every 25ms

   if(FASTEST_PATH){//turn on PID tuning if fastest path
     PIDControlStraight.SetTunings(11,0.1, 1.5);
     PIDControlStraight.SetSampleTime(15);
   }
   PIDControlStraight.SetMode(AUTOMATIC); //Controller is invoked automatically using default value for PID
   Serial.println("PID controller straight: ");
   PIDdebug(PIDControlStraight);


   while(tick_travelled < target_tick){
      // if not reach destination ticks yet
      currentTick1 = tick1 - oldTick1; //calculate the ticks travelled in this sample interval of 50ms
      currentTick2 = tick2 - oldTick2;
      
//      Serial.print(currentTick1); //for debug
//      Serial.print(" "); Serial.println(currentTick2);
      PIDControlStraight.Compute();

      oldTick2 += currentTick2; //update ticks
      oldTick1 += currentTick1;
      tick_travelled += currentTick2;
      //oldTick2 = tick2;
      //oldTick1 = tick1;
//      if (int(tick_travelled) == 300 || int(tick_travelled) == 600 || int(tick_travelled) == 900 ||int(tick_travelled) == 1200 ||int(tick_travelled) == 1500 || int(tick_travelled) == 1800 ||int(tick_travelled) == 2100 || int(tick_travelled) == 2400 || int(tick_travelled) == 2700)
//        Serial.println(tick_travelled);
//        Serial.println("MC");
//        delay(10);
      
   }
   
   for (int i = 0; i <= 400; i+=50){
     md.setBrakes(i-5,i+5);
     delay(2.5); 
   }
   PIDControlStraight.SetMode(MANUAL);
   Serial.println("PID controller straight END: ");
   PIDdebug(PIDControlStraight);
   Serial.println("MC");
   delay(5);
   if(FASTEST_PATH)
      delay(delayFastestPath);
}

/*
 * ==============================================================
 * Sensors
 * Methods dealing with data acquisition and processing of sensor
 * ==============================================================
 */

 //Methods to return blocks unit from obstacles ++++++++++++
int distance_short_front_center(){
  int distance_cm = 0;
  for(int i=SAMPLE; i>0; i--){
      sensor_reading[i] = front_center.getDistance(true);
      delay(1);
    }

  sortArray(sensor_reading,SAMPLE);
    
  //  sensor_distance_cm[0] = sensor_reading[SAMPLE/2];
  //  Serial.println(sensor_distance_cm[0]);
  
  distance_cm = sensor_reading[SAMPLE/2];
  if (distance_cm <=12)
    return 1;//org 0
  else if (distance_cm <=22)
    return 2; //org 1
//  else if (distance_cm <= 33)
//    return 3; //org 2
  else
    return -1;
  
}


int distance_short_front_left(){
  int distance_cm = 0;
  int distance_blocks = 0;
  for(int i=SAMPLE; i>0; i--){
      sensor_reading[i] = front_left.getDistance(true);
      delay(1);
    }
  sortArray(sensor_reading,SAMPLE);
//  sensor_distance_cm[1] = sensor_reading[SAMPLE/2];
//  if (DEBUG)
//  Serial.println(sensor_distance_cm[1]);

  distance_cm = sensor_reading[SAMPLE/2];

  if (distance_cm <= 15)
    return 1;
  else if (distance_cm <= 24)
    return 2;
//  else if (distance_cm <= 37)
//    return 3;
  else return -1;
  

    
}

int distance_short_front_right(){
  int distance_cm = 0;
  int distance_blocks = 0;
  
  for(int i=SAMPLE; i>0; i--){
    sensor_reading[i] = front_right.getDistance(true);
    delay(1);
  }  

  sortArray(sensor_reading,SAMPLE);

  //sensor_distance_cm[2] = sensor_reading[SAMPLE/2];
  //if (DEBUG)
  //Serial.println(sensor_distance_cm[2]);

  distance_cm = sensor_reading[SAMPLE/2];

  if (distance_cm <= 15)
    return 1;
  else if (distance_cm <= 24)
    return 2;
//  else if (distance_cm <= 29)
//    return 3;
  else 
    return -1;
  
  

}

int distance_short_right_front(){
  for(int i=SAMPLE; i>0; i--){
    sensor_reading[i] = right_front.getDistance(true);
    delay(1);
  }

  sortArray(sensor_reading,SAMPLE);

  //sensor_distance_cm[3] = sensor_reading[SAMPLE/2];
  distance_cm = sensor_reading[SAMPLE/2];
  if (distance_cm <= 15)
    return 1; //org 0
  else if (distance_cm <=25)
    return 2; //org 1
//  else if (distance_cm <= 34)
//    return 3;
  else 
    return -1;
  
   
}

int distance_short_right_back(){
  for(int i=SAMPLE; i>0; i--){
    sensor_reading[i] = right_back.getDistance(true);
    delay(1);
  }

  sortArray(sensor_reading,SAMPLE);

//  sensor_distance_cm[4] = sensor_reading[SAMPLE/2];
//
//  if (DEBUG)
//  Serial.println(sensor_distance_cm[4]);

  distance_cm = sensor_reading[SAMPLE/2];
  if (distance_cm <= 17)
    return 1; //org 0
  else if (distance_cm <=28)
    return 2; //org 1
//  else if (distance_cm <=37)
//    return 3;
  else 
    return -1;
}

int distance_long_left(){
  for(int i=SAMPLE; i>0; i--){
    sensor_reading[i] = long_left.getDistance(true);
    delay(1);
  }

  sortArray(sensor_reading,SAMPLE);

//  sensor_distance_cm[5] = sensor_reading[SAMPLE/2];
//
//  if (DEBUG)
//  Serial.println(sensor_distance_cm[5]);

  distance_cm = sensor_reading[SAMPLE/2];
  if (distance_cm <= 19)
     return 1; //org 0
  else if (distance_cm <= 24)
     return 2; //org 1
  else if (distance_cm <= 32)
    return 3; //2 
  else if (distance_cm <= 41)
    return 4; //3
  else if (distance_cm <= 50)
    return 5; //4
  else
    return -1;
  
}

void read_all_sensors(int delay_time){
  delay(delay_time);
  int sensor1,sensor2,sensor3,sensor4,sensor5,sensor6;
  sensor1 = distance_short_front_center();
  sensor2 = distance_short_front_left();
  sensor3 = distance_short_front_right();
  sensor4 = distance_short_right_front();
  sensor5 = distance_short_right_back();
  sensor6 = distance_long_left();
  Serial.print("SD|");
  Serial.print(sensor1); Serial.print(";");
  Serial.print(sensor2); Serial.print(";");
  Serial.print(sensor3); Serial.print(";");
  Serial.print(sensor4); Serial.print(";");
  Serial.print(sensor5); Serial.print(";");
  Serial.println(sensor6);
  
}




//+++++++++++++++++++++++++++++++++++++++++

//Methods for detect object in front of sensors, for clearance purpose ++++++++++++++++++++++++++++

bool has_obstacle_front_center(){
  if (distance_short_front_center() == 1 && distance_short_front_center()!=-1 )
    return true;
  else
    return false;
}

bool has_obstacle_front_left(){
  if (distance_short_front_left() != -1)
    return true;
  else
    return false;
}

bool has_obstacle_front_right(){
  if (distance_short_front_right() != -1)
    return true;
  else
    return false;
}

bool has_obstacle_right_front(){
  if (distance_short_right_front() != -1)
    return true;
  else
    return false;
}

bool has_obstacle_right_back(){
  if (distance_short_right_back() != -1)
    return true;
  else
    return false;
}

bool has_obstacle_long_left(){
  if (distance_long_left() == 1)
    return true;
  else
    return false;
}
//+++++++++++++++++++++++++++++++++++++++++
/*
 * ==========================================================================
 * Calibration
 * Methods to align wall, calibrate by front sensors, for turning at corner
 * ==========================================================================
 */

//Method for right wall alignment
void right_wall_calibrate(){
  double difference = 0;
  double distance_front = 0;
  double distance_back = 0;
  boolean distance_calibrate_only = false;
  int i = 35;

  //calibrate distance first by front calibrate
  if (!has_obstacle_right_front() && !has_obstacle_right_back()){
    if (DEBUG) Serial.println("No Wall");
    return;
  }
  else if(!has_obstacle_right_front() || !has_obstacle_right_back()){
    if (distance_short_right_front() != distance_short_right_back()){
        distance_calibrate_only = true;
        i = 0;
    }
    Serial.println(distance_calibrate_only);
    
  }
  if (DEBUG) Serial.println("Right Wall distance calibration");
  for (int j = 10; j>0; j--){
      distance_front += right_front.getDistance(true);
      distance_back += right_back.getDistance(true);
    }
  distance_front /= 10;
  distance_back /= 10;

  if(distance_front <10.5 || distance_back <10.5){ 
    //If robot is too close to the rightwall
      rotate_right(90);
      delay(50);
      front_calibrate();
      delay(20);
      rotate_left(90);
      delay(20);
    }

   else if (distance_front > 12 || distance_back > 12){
      rotate_right(90);
      delay(50);
      front_calibrate();
      delay(20);
      rotate_left(90);
      delay(20);
   }
   
   //otherwise only need to calibrate angle   
  if (DEBUG){ 
    Serial.println("Right Wall Angle Calibration");
    Serial.println("Distance calibrate only");
    Serial.println(distance_calibrate_only);
  }
  while(i > 0 && distance_calibrate_only == false){
    for (int j = 10; j>0; j--){
      distance_front += right_front.getDistance(true);
      distance_back += right_back.getDistance(true);
    }
    distance_front /= 10;
    distance_back /= 10;
    
    difference = (distance_front - distance_back);
    
    if(DEBUG){
      Serial.println(" ");
      Serial.print("distance_front: ");
      Serial.print(distance_front);
      Serial.print("distance_back: ");
      Serial.println(distance_back);
      Serial.print("difference");
      Serial.println(difference);
    }
    delay(1);
    i--;                                          
    

    if(difference >= 0.03 && distance_back < 25){ //If the robot tilts to the right 
      md.setSpeeds(rpm_to_speed_1(-20),rpm_to_speed_2(20));
      delay(15);
      md.setBrakes(50,50);
    }
    
    else if(difference <= -0.03 && distance_back  < 25){ //If the robot tilts to the left
      md.setSpeeds(rpm_to_speed_1(20),rpm_to_speed_2(-20));
      delay(15);
      md.setBrakes(50,50);
    }
    
    else{ // If difference is in between 0.035 to -0.035
      break;
    }
  }
  md.setBrakes(100,100);
  if(DEBUG) Serial.println("Done Side calibration");
  delay(10);
  
}

//Method to calibrate using front sensors
void front_calibrate(){
  double distance_left = 0;
  double distance_right = 0;
  double difference = 0;
  double ideal = 11;
  int k = 20;
  int j = 2;
  bool only_distance_calibrate = false;

  if (!has_obstacle_front_left() && !has_obstacle_front_right())
    return;
  
  else if(!has_obstacle_front_left() || !has_obstacle_front_right()){
      //Only calibrate distance if there is an empty block in front on either side
      if(DEBUG) Serial.println("Not enough objects, distance calibration kickinng in!");
      only_distance_calibrate = true;
      k = 0;
   }
  else {//when both has obstacle in front, check how far is the obstacle
      if (distance_short_front_center() != distance_short_front_left() ||distance_short_front_center() != distance_short_front_right()){ 
          only_distance_calibrate = true;
          k = 0;
      } 
  }
   
  if(DEBUG)
    Serial.println("Front calibrating");
  while (j>0){
    while (k>0 && only_distance_calibrate == false){
      //calibrate in term of angles first
  
      //Step1: Collecting Data
      for (int i = 10; i>0; i--){
        distance_left += front_left.getDistance(true);
        distance_right += front_right.getDistance(true);
      }
      distance_left /= 10;
      distance_right /= 10;
  
      //Step 2: Calculate the difference
      difference  = distance_left - (distance_right);
  
      //Some debug prints
      if (DEBUG){
        Serial.print(distance_left);
        Serial.print("|");
        Serial.print(distance_right);
        Serial.print("|");
        Serial.println(difference);
      }
      k--;
      delay(1);
      //calibrate the angle by rotate left/right
      if (difference > 0.03){
        k++;
        md.setSpeeds(rpm_to_speed_1(-20),rpm_to_speed_2(20));
        delay(10);
        md.setBrakes(50,50);
      }
  
      else if(difference < -0.03){
        k++;
        md.setSpeeds(rpm_to_speed_1(20),rpm_to_speed_2(-20));
        delay(10);
        md.setBrakes(50,50);
      }
  
      else
        break;
    }
    if(DEBUG && only_distance_calibrate == false)
      Serial.println("Done calibrating angle front");
    
    k = 15;
    
    //calibrating distance
    while (k>0){
      //collect data
      for (int i = 10; i>0; i--){
          distance_left += front_left.getDistance(true);
          distance_right += front_right.getDistance(true);
          }
      distance_left /= 10;
      distance_right /= 10;
      if (DEBUG){
        Serial.print(distance_left);
        Serial.print("|");
        Serial.println(distance_right);
      }  
      k--;
      if (distance_left < ideal || distance_right < ideal){
        md.setSpeeds(rpm_to_speed_1(-20),rpm_to_speed_2(-20));
        delay(15);
        md.setBrakes(50,50);
      }
  
      else if (distance_left > ideal || distance_right > ideal)
      {
        md.setSpeeds(rpm_to_speed_1(20),rpm_to_speed_2(20));
        delay(15);
        md.setBrakes(50,50);
      }
      else
        break;
  
  
    }
    if (DEBUG){
      Serial.println("Done with distance front calibration");
    }
    j--;
    k=15;
  }
  if (DEBUG){
    Serial.println("Done front calibration!");
  }
  delay(10);
    
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
 void PIDdebug(PID controller){
  Serial.println("PID constants (P, I, D): ");
  Serial.println(controller.GetKp());
  Serial.println(controller.GetKi());
  Serial.println(controller.GetKd());
  Serial.println(controller.GetMode());
 }
