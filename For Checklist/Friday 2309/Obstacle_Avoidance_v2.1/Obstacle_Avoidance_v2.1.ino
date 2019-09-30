#include "DualVNH5019MotorShield.h"
#include "SharpIR.h"
#include "EnableInterrupt.h"

DualVNH5019MotorShield md;
SharpIR ps1_long(SharpIR:: GP2Y0A02YK0F, A0);
SharpIR ps2_short_Front_Left(SharpIR:: GP2Y0A21YK0F, A1);
SharpIR ps3_short_Front_Right(SharpIR:: GP2Y0A21YK0F, A2);
SharpIR ps4_short_Right(SharpIR:: GP2Y0A21YK0F, A3);


/*
 * This code is meant to complete checklist A.6 and is the primary blueprint for the final project.
 * Notes:
 * 1. right_Calibrate() remains incomplete as it is not useful - waiting for news about the purpose of the last two sensors that are not added yet.
 * 2. The agenda of version 2 is to document the coding practices used on this code, as well as changing anything that violates the mentioned practice as accordingly.
 * 3. Version 2 will also include a brief logic description of the entiure and the logic description of all the functions. 
 * 
 * Updates on 20/9:
 * 1. Changing the approach of the PID - we will be using the PID library available in Arduino
 * 2. Sent a message whenever an obstacle is detected
 * 3. Allow external software to send commands to control the movement of the robot
 * 4. Sent a acknowledgement whenever an action is done.
 * 
 * Agenda for v2.1:
 * 1. Implement the updates mentioned.
 * 
 * Coding practices:
 * "/" represents or for the following sentences.
 * 1. For all units(ticks, RPM, PWM, distance) that are related to the Motor(no.1/2) or Encoder(no.1/2) - It should be declared as "M/E(1/2)_unit" (E.g. E1_ticks).
 * 2. Additional words can be added to the above practice in any order but must in small letters (E.g. E1_ticks_moved, inverted_M2_speed_rpm).
 * 3. All functions are called in this manner - "M/E/PID/single_Singleword_UNIT_1()". (E.g. print_Converted_PWM()) with the exception of PID_Controller(), E1_ticks_increment(), E2_ticks_increment().
 * 4. All variables that only that does not use short form words are to be declared with only small letters such as  - "singleword_singleword".
 * 5. All pin definition are declared in this manner - partIdentifier_INPUT/OUTPUT. (E.g. encoder1A_INPUT) with the exception of enable_M1 and enable_M2.
 *    
 * Code Description:
 * 1. Run forward() in the loop.
 * 2. forward() will run front_Calibrate() to check the sensors constanly every 1 second.
 * 3. front_Calibrate() will run PID_controller() if there are no obstacles ahead, otherwise it will brake, turn left or right according to what the robot sense.
 * 4. If it brakes in front_Calibrate(), front_Calibrate() stops and returns to forward().
 * 5. forward() will then run right_Calibrate() to check the right side of the robot. - **Logic remains incomplete here.
 */


//Motor 1
#define encoder1A_INPUT 3
#define encoder1B_INPUT 5
#define motor1A_OUTPUT 2
#define motor1B_OUTPUT 4
#define enable_M1 6

//Motor 2
#define encoder2A_INPUT 11
#define encoder2B_INPUT 13
#define motor2A_OUTPUT 7
#define motor2B_OUTPUT 8
#define enable_M2 12

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
int current_direction = 0;
unsigned long start_time = 0;

void setup()
{
  Serial.begin(9600);
  md.init();


  //Motor 1
  digitalWrite(enable_M1, HIGH);
  
  pinMode(motor1A_OUTPUT, OUTPUT);
  pinMode(motor1B_OUTPUT, OUTPUT);
  
  pinMode(encoder1A_INPUT, INPUT);
  pinMode(encoder1B_INPUT, INPUT);
  pulseIn(encoder1A_INPUT, HIGH);



  //Motor 2
  digitalWrite(enable_M2, HIGH);
  
  pinMode(motor2A_OUTPUT, OUTPUT);
  pinMode(motor2B_OUTPUT, OUTPUT);
  
  pinMode(encoder2A_INPUT, INPUT);
  pinMode(encoder2B_INPUT, INPUT);
  pulseIn(encoder2A_INPUT, HIGH);

  //Interrupts
  enableInterrupt(encoder1A_INPUT, E1_ticks_increment, RISING);
  enableInterrupt(encoder2A_INPUT, E2_ticks_increment, RISING);
}

void loop ()
{ 
  char command;
  while(Serial.available()){
    command = Serial.read();
    if(command == 'w'){
      forward(1);
    }
    if(command == 'a'){
      left(90);
    }
    if(command == 'd'){
      right(90);
    }
    if(command == 'x'){
      setBrakes(400, 400);
    }
  }
  forward();
  delay(100);
}

/*
 * Function logic for right():
 * 1. Take the number of grids the robot should turn = grids_to_move
 * 2. Convert it to ticks that the motor should move.
 * 3. Checks if the ticks moved over shot the amount of ticks the motor should move.
 * 4. Else keeps running the front_Calibrate()
 */
void forward(int grids_to_move){
  int ticks_to_move = ((grids_to_move*10)/6) * 562.25;
  E1_ticks = 0;
  E2_ticks = 0;
  E1_ticks_moved = 0;
  E2_ticks_moved = 0;
  while(1){
    if(E1_ticks_moved > ticks_to_move || E2_ticks_moved > ticks_to_move){
      md.setBrakes(-400, 400);
      delay(100);
      md.setBrakes(0, 0);
      break;
    }
    front_Calibrate();
    delay(1000);
    //total delay in this loop should be 1 second, since the PID feedback is at its most accurate when PID_Controller is called every 1 second.
  }
}

/*
 * Function logic for front_Calibrate():
 * While the distance obtained from both left and right short sensors in the front is not entering the blind spot:
 * 1. Checks if there are no obstacles in front  - runs PID controller function that provides the forward direction identifier.
 * 2. Else checks if obstacles had already entered the blind spot - robot reverse.
 * 3. Else checks if there are incoming obstacle from the front left side - robot brakes and turn right.
 * 4. Else checks if there are incoming obstacle from the front right side - robot brakes and turn left.
 * If an obstacle enters the blind spot of the sensor - robot brakes and breaks the whole loop.
 */
void front_Calibrate(){

  Serial.println("Front Calibrating");
  
  while(ps2_short_Front_Left.getDistance() != 12 || ps3_short_Front_Right.getDistance() != 12){
    
    //print_Distance();

    //Detect obstacle 2 grids away for short sensor, 5 grids away for long sensor and sent a message to RaspberryPI
    sensor_Reading();
    
    //No obstacle straight in front
    if(ps2_short_Front_Left.getDistance() > 12 && ps3_short_Front_Right.getDistance() > 12){
     PID_Controller(1);
    }
    //Wide obstacle straight in front
    else if(ps2_short_Front_Left.getDistance() < 12 && ps3_short_Front_Right.getDistance() < 12){
      md.setSpeeds(-100, -100);
    }
    //Incoming obstacle on the left
    else if(ps2_short_Front_Left.getDistance() < 12 && ps3_short_Front_Right.getDistance() > 12){
       md.setBrakes(400, 400);
       right(75);
    }
    //Incoming obstacle on the right
    else if(ps2_short_Front_Left.getDistance() > 12 && ps3_short_Front_Right.getDistance() < 12){
       md.setBrakes(400, 400);
       left(75);
    }
  }
  md.setBrakes(400, 400);
  delay(100);
}

void sensor_Reading(){

  distance_long = ps1_long.getDistance();
  grid_long = distance_to_grid(distance_long);
  if(grid_SFL <= 5){
    Serial.println("Obstacle directly in front: 5 grids");
  }
  
  distance_SFL = ps2_short_Front_Left.getDistance();
  grid_SFL = distance_to_grid(distance_SFL);
  if(grid_SFL <= 2){
    Serial.println("Obstacle front-left: 2 grids");
  }

  distance_SFR = ps3_short_Front_Right.getDistance();
  grid_SFR = distance_to_grid(distance_SFR);
  if(grid_SFR){
    Serial.println("Obstacle front-right: 2 grids");
  }
}

//This right_Calibrate function will only run after the robot stops moving from front_Calibrate().
//Currently the logic behind right_Calibrate() is that the robot will be keep turning left until there is no obstacle on the top right-side of the robot.
void right_Calibrate(){

  Serial.println("Right calibrating.");

   while(ps4_short_Right.getDistance() != 12){
     if(ps4_short_Right.getDistance() < 12){
        left(75);
     }
   }
}


/*
 * Function logic for right():
 * 1. Take the degree the robot should turn = degree_to_move
 * 2. Convert it to ticks that the motor should move.
 * 3. Checks if the ticks moved over shot the amount of ticks the motor should move.
 * 4. Else keeps running the PID controller to move the robot, in which the PID controller function provides the left direction identifier.
 */
void right(int degree_to_move){

  //Travel distance variables in Ticks
  int ticks_to_move = 0;
  E1_ticks = 0;
  E2_ticks = 0;
  E1_ticks_moved = 0;
  E2_ticks_moved = 0;

  if (degree_to_move == 90){
    ticks_to_move = 450;
  }
  if (degree_to_move == 75){
    ticks_to_move = 375;
  }
  
  while(1){
    if(E1_ticks_moved > ticks_to_move || E2_ticks_moved > ticks_to_move){
      md.setBrakes(-400, 400);
      delay(100);
      md.setBrakes(0, 0);
      break;
    }
    PID_Controller(4);
  }
}


/*
 * Function logics for right():
 * 1. Take the degree the robot should turn = degree_to_move
 * 2. Convert it to ticks that the motor should move.
 * 3. Checks if the ticks moved over shot the amount of ticks the motor should move.
 * 4. Else keeps running the PID controller to move the robot, in which the PID controller function provides the right direction identifier.
 */
void left(int degree_to_move){
  int ticks_to_move = 0;
  E1_ticks = 0;
  E2_ticks = 0;
  E1_ticks_moved = 0;
  E2_ticks_moved = 0;

  if (degree_to_move == 90){
    ticks_to_move = 450;
  }
  if (degree_to_move == 75){
    ticks_to_move = 375;
  }
  
  while(1){
    if(E1_ticks_moved > ticks_to_move || E2_ticks_moved > ticks_to_move){
      md.setBrakes(400, -400);
      delay(100);
      md.setBrakes(0, 0);
      break;
    }
    PID_Controller(3);
  }
}

/*-----PID-----*/
/*Function logic for PID_Controller():
 * 1. Get period
 * 2. Convert ticks incremented to the current rpm of the motors
 * 3. Calculate the error between the setpoint rpm and the current rpm
 * 4. Calculate the next speed input for the motor in rpm.
 * 5. Limit the next speed input so that it does not exceed pre-determined rpm range.
 * 6. Convert the next speed input from rpm to pwm.
 * 7. Depending on the received direction identifier, set the speed of the motor from the converted pwm.
 * 8. Calculate the amount of ticks that moved.
 * 9. Reset the ticks incremented.
 * 10. Update the error variables required for the PID calculation.
 */
//https://projects.raspberrypi.org/en/projects/robotPID/5
void PID_Controller(int directionflag)
{
  //Debug log to check if PID is running.
  //Serial.println("");
  //Serial.println("PID is running.");

  //Debug log to see ticks generated.
  print_TICKS();

  //Get Period and Debug log to see period.
  double period = (double)(millis()-start_time)/1000;
  //Serial.print("Sampling interval: ");
  //Serial.println(period);

  /*-----Convert Ticks to RPM-----*/
  E1_rpm = (((double)E1_ticks/562.25)/period)*60;
  E2_rpm = (((double)E2_ticks/562.25)/period)*60;

  //Debug log to see RPM converted from Ticks from Encoders.
  //print_Converted_RPM();


  
  /*-----PID Calculation using RPM-----*/
  E1_error_rpm = M1_setpoint_rpm - E1_rpm;
  E2_error_rpm = M2_setpoint_rpm - E2_rpm;
  //print_Errors();
  
  M1_speed_rpm += (E1_error_rpm * KP1) + (E1_prev_error_rpm * KD1);// + (E1_sum_error_rpm * KI1);
  M2_speed_rpm += (E2_error_rpm * KP2) + (E2_prev_error_rpm * KD2);// + (E2_sum_error_rpm * KI2);
  //Motor 1 usually goes faster than Motor 2.

  /*-----Limit-----*/
  M1_speed_rpm = max(min(60, M1_speed_rpm), 0);
  M2_speed_rpm = max(min(60, M2_speed_rpm), 0);
  
  //Debug log to see RPM calculated from PID.
  //print_Feedback_RPM();

  
  /*-----Convert RPM to PWM-----*/
  M1_speed_pwm = RPM_To_PWM_1(M1_speed_rpm);
  M2_speed_pwm = RPM_To_PWM_2(M2_speed_rpm);
  inverted_M1_speed_pwm = -M1_speed_pwm;
  inverted_M2_speed_pwm = -M2_speed_pwm;

  //Debug log to see PWM converted from RPM calulated from PID.
  //print_Converted_PWM();

  
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
  start_time = millis();

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

double RPM_To_PWM_1(double RPM){
  double pwmMotor1 = 2.7037*RPM + 39.27;
  return pwmMotor1;
}

double RPM_To_PWM_2(double RPM){
  double pwmMotor2 = 2.7037*RPM + 34.236;
  return pwmMotor2;
}



/*-----Serial Prints-----*/
void print_Converted_RPM(){
  //Serial.println("");
  //Serial.print("E1_rpm: ");
  Serial.print(E1_rpm);
  Serial.print(" ");
  //Serial.print("   E2_rpm: ");
  Serial.println(E2_rpm);
}

void print_TICKS(){
  Serial.println("");
  Serial.print("E1_ticks: ");
  Serial.print(E1_ticks);
  Serial.print("     E2_ticks: ");
  Serial.println(E2_ticks);
}

void print_Feedback_RPM(){
  Serial.println("");
  Serial.print("M1_speed_rpm: ");
  Serial.print(M1_speed_rpm);
  Serial.print("   M2_speed_rpm: ");
  Serial.println(M2_speed_rpm);
}

void print_Converted_PWM(){
  Serial.println("");
  Serial.print("M1_speed_pwm: ");
  Serial.print(M1_speed_pwm);
  Serial.print("   M2_speed_pwm: ");
  Serial.println(M2_speed_pwm);
}

void print_Errors(){
  Serial.println("");
  Serial.print("E1_error_rpm: ");
  Serial.print(E1_error_rpm);
  Serial.print("   E2_error_rpm: ");
  Serial.println(E2_error_rpm);
}

void print_Distance(){
  Serial.println(" ");
  Serial.print("ps2_short_Front_Left: ");
  Serial.print(ps2_short_Front_Left.getDistance());
  Serial.print("  ps3_short_Front_Right: ");
  Serial.println(ps3_short_Front_Right.getDistance());
  Serial.print("  ps1_longt: ");
  Serial.println(ps1_long.getDistance());
}
