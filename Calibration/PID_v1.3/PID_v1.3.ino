#include "DualVNH5019MotorShield.h"
#include "EnableInterrupt.h"

DualVNH5019MotorShield md;

//Motor 1
#define encoder1A 3
#define encoder1B 5

//Motor 2
#define encoder2A 11
#define encoder2B 13

#define SAMPLETIME 1 //in seconds how often is read
/*-----Ticks Variable-----*/
volatile long E1_ticks = 0;
volatile long E2_ticks = 0;

/*-----Check Ticks Variables-----*/
double E1_ticks_moved = 0;
double E2_ticks_moved = 0;
double ticks_to_move = 0;

/*-----Motor Speed Variables-----*/
double M1_speed = 0;
double M2_speed = 0;
double M1_ticks_PID = 0;
double M2_ticks_PID = 0;

/*-----PID Variables-----*/
double M1_setpoint_ticks = 0;
double M2_setpoint_ticks = 0;
double E1_error_ticks = 0;
double E2_error_ticks = 0;
double E1_prev_error = 0;
double E2_prev_error = 0;
double E1_sum_error = 0; //16919;
double E2_sum_error = 0; //17198;
double KP = 0.2;
double KD = 0.01;
double KI = 0.005;

double targetRPM = 70;

bool flag = false;

void setup()
{
  Serial.begin(9600);
  md.init();
  
  pinMode(encoder1A, INPUT);
  pinMode(encoder1B, INPUT);
  pulseIn(encoder1A, HIGH);

  pinMode(encoder2A, INPUT);
  pinMode(encoder2B, INPUT);
  pulseIn(encoder2A, HIGH);

  //md.setSpeeds(200, 200);
  md.setSpeeds(RPMtoSpeedM1(targetRPM),RPMtoSpeedM2(targetRPM));
  enableInterrupt(encoder1A, E1_ticks_increment, RISING);
  enableInterrupt(encoder2A, E2_ticks_increment, RISING);

}

void loop ()
{
  
  while(Serial.available())
  {
    char command;
    command = Serial.read();
    if(command == 'w')  
    {
      Serial.println("I received w.");
      moveforward();
      flag = true;
    }
    if(command == 'a')  
    {
      Serial.println("I received a.");
      Serial.print("E1_sum_error: ");
      Serial.println(E1_sum_error);
      Serial.print("E2_sum_error: ");
      Serial.println(E2_sum_error);
      md.setBrakes(400, 400);
      flag = false;
    }
  }


  if(flag == true){
      PIDController();
      delay(SAMPLETIME*1000); //get called every 1 seconds
  }
}

void moveforward()
{
  M1_setpoint_ticks = 200;
  M2_setpoint_ticks = 200;

  //Probably redundant
  ticks_to_move = 1000;
  if(E1_ticks_moved > ticks_to_move || E2_ticks_moved > ticks_to_move)
  {
    Serial.println("Braking");
    md.setBrakes(400, 400);
    delay(100);
  }
  
  PIDController();
}


/*-----PID-----*/
//https://projects.raspberrypi.org/en/projects/robotPID/5
void PIDController()
{
  Serial.println("PID is running");
  Serial.print("Actual E1_ticks: ");
  Serial.print(E1_ticks);
  Serial.print("     E2_ticks: ");
  Serial.println(E2_ticks);
  
  E1_error_ticks = M1_setpoint_ticks - E1_ticks;
  E2_error_ticks = M2_setpoint_ticks - E2_ticks;
  
  M1_ticks_PID = E1_ticks + (E1_error_ticks * (KP * 0.95)) + (E1_prev_error * (KD + 0.01)) + (E1_sum_error * KI);
  M2_ticks_PID = E2_ticks + (E2_error_ticks * KP) + (E2_prev_error * KD) + (E2_sum_error * KI);

  //convert adjusted ticks to RPM
  M1_speed = RPMtoSpeedM1(ticks_to_rpm(M1_ticks_PID));
  M2_speed = RPMtoSpeedM2(ticks_to_rpm(M2_ticks_PID));

  md.setSpeeds(M1_speed, M2_speed);

  E1_ticks_moved += E1_ticks;
  E2_ticks_moved += E2_ticks;
      
  Serial.print("M1_speed: ");
  Serial.print(M1_speed);
  Serial.print("     M2_speed: ");
  Serial.println(M2_speed);
      
  //Reset 
  E1_ticks = 0;
  E2_ticks = 0;

  E1_prev_error = E1_error_ticks;
  E2_prev_error = E2_error_ticks;

  E1_sum_error += E1_error_ticks;
  E2_sum_error += E2_error_ticks;
}


void E1_ticks_increment()
{
  E1_ticks ++;
}

void E2_ticks_increment()
{
  E2_ticks ++;
}

double RPMtoSpeedM1(double rpm1 ){
  return 2.7037*rpm1 + 41.27;
}

double RPMtoSpeedM2(double rpm2 ){
  return 2.6872*rpm2 + 34.236;
}

double ticks_to_rpm(unsigned long tick){
  return (((double)tick*2/2249)/SAMPLETIME*1000000*60);
}
