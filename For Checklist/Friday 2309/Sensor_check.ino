#include "SharpIR.h"

/*
 * This Arduino code is meant to complete MDP assessment checklist A.3 "Sensors calibrated to correctly return distance to obstacle."
 * 
 * Description:
 * Using either any combination of IR and/or ultrasonic sensors, demonstrate that
 * you can correctly return the distance to two targets (obstacle blocks) selected by
 * your supervisor.
 * The first target will be placed between 30-50cm away, and the second target
 * will be placed between 50-70 cm away. Your range readings must be accurate
 * to within +/- 6%.
 * 
 * Experiment conditions:
 * 1. All measurements are done with the ion battery turned on, turning it on and off has an actual effect to the readings
 * 2. Distance measured is between the obstacle and cross section of the side of the robot that is on a straight line to the sensor.
 * 3. For long sensor, calibration was done starting from 30cm to 70cm
 * 4. For short sensor, calibration was done starting from 20cm to 30cm
 */
SharpIR ps1_short_Front_Left(SharpIR:: GP2Y0A21YK0F, A0);
SharpIR ps2_short_Back_Left(SharpIR:: GP2Y0A21YK0F, A1);
SharpIR ps3_short_Left(SharpIR:: GP2Y0A21YK0F, A2);
SharpIR ps4_short_Front_Right(SharpIR:: GP2Y0A21YK0F, A3);
SharpIR ps5_long(SharpIR:: GP2Y0A02YK0F, A4);
SharpIR ps6_short_Right(SharpIR:: GP2Y0A21YK0F, A5);

int samplesize;
double distance;
double result;
int offset;

void setup() {
  Serial.begin(9600);

  samplesize = 0;
  distance = 0;
  result = 0;
  offset = 0;
}

void loop() {

  /* ps1 does not work as intended
   * sensor at the back work instead - 
   * sensor at the back work on both ps1_short_Front_Left.getDistance & ps2_short_Back_Left.getDistance()
   */
  distance = ps2_short_Back_Left.getDistance();
  Serial.println(distance);


  //ps2_SBL();
  //ps3_SL();
  //ps4_SFR();
  //ps5();
  //ps6_SR();
  delay(1);
}

void ps5(){
  distance += ps5_long.getDistance();
  samplesize ++;
  if(samplesize == 1000){
    result = distance/1000;
    //70cm -> avg = 71 to 74, offset = -3, likely to fall short or exceed this range
    if(result > 72){
      offset = -3;
    } 
    //68cm -> avg = 69.06 to 71.8, offset = -1.5, will rarely fall short or exceed this range,
    else if(result >= 68 && result < 72){
      offset = -1.5;
    } 
    //65cm -> avg = 64 to 66, offset = 0, will rarely fall short or exceed this range, is unreliable from starting from this distance 
    else if(result >= 64 && result < 68){
      offset = 0;
    } 
    //63cm -> avg = 61.43 to 63.3, offset = 1
    else if(result >= 60 && result < 64){
      offset = 1;
    }
    //60cm -> avg = 55.9 to 58.3, offset = 3, will rarely fall short or exceed this range
    else if (result >= 54 && result < 60){
      offset = 3;
    }
    //56cm -> avg = 50.5 to 51.78, offset = 5
    else if (result >= 49 && result < 54){
      offset = 5;
    }
    //52cm -> avg = 46.1 to 47.1, offset = 6
    //46cm -> avg = 40.25 to 40.41, offset = 6
    else if (result >= 40 && result < 49){
      offset = 6;
    }
    //44cm -> avg = 38.45 to 38.57, offset = 5.5
    else if (result >= 38 && result < 40){
      offset = 5.5;
    }
    //42cm -> avg = 36.78 to 36.92, offset = 5
    //40cm -> avg = 34.88 to 34.91, offset = 5
    else if (result >= 34 && result < 38){
      offset = 5;
    }
    //38cm -> avg = 32.9 to 32.94, offset = 6
    else if (result >= 32 && result < 34){
      offset = 6;
    }
    //36cm -> avg = 30.9 to 31.1, offset = 5
    else if (result >= 30 && result < 32){
      offset = 5;
    }
    //34cm -> avg = 29.3 to 29.45, offset = 4
    //33cm -> avg = 28.6 to 28.8, offset = 4
    //32cm -> avg = 27.74 to 27.9, offset = 4
    //30cm -> avg = 25.95 to 25.97, offset = 4 
    else if (result < 30){
      offset = 4;
    }
    //20cm -> avg = 24
    Serial.println(result + offset);
    samplesize = 0;
    distance = 0;
  }
}

void ps4_SFR(){
  distance += ps4_short_Front_Right.getDistance();
  samplesize ++ ;
  if(samplesize == 1000){
    result = distance/1000;
    samplesize = 0;
    //70cm -> avg = 79 to 81
    if(result > 79){
      offset = -10;
    }
    else if (result >= 77 && result < 79) {
      offset = -11;
    }
    //60cm -> avg = 70 to 76, higher freqency of 72
    else if (result >= 69 && result < 77){
      offset = -12;
    }
    //55cm -> avg = 66 to 69
    else if (result >= 62 && result < 69){
      offset = -11;
    }
    //Need to use wide obstacle
    //50cm -> avg = 59.7 to 60.7
    else if (result >= 59 && result < 62){
      offset = -10;
    }
    //45cm -> avg = 56 to 56.9
    else if (result >= 55 && result < 59){
      offset = -12;
    }
    //40cm -> avg = 49.8 to 50.1 
    else if (result >= 47 && result < 55){
      offset = -10;
    }
    //37cm -> avg = 44.6 to 45.6
    else if (result >= 43 && result < 47){
      offset = -8.5;
    }
    //35cm -> avg = 41.8 to 42.1
    else if (result >= 40 && result < 43){
      offset = -7;
    }
    //30cm -> avg = 35.8 to 35.9
    else if (result >= 33 && result < 40){
      offset = -6;
    }
    //27cm -> avg = 31.8 to 31.9
    //24cm -> avg = 28.8 to 28.9
    else if (result >= 24 && result < 33){
      offset = -5;
    }
    //20cm -> avg = 23.9 to 24
    else if (result >= 14 && result < 24){
      offset = -4;
    }
    //10cm -> avg = 13
    else if (result < 14){
      offset = -3;
    }
    Serial.println(result + offset);
    distance = 0;
  }  
}

void ps6_SR(){
  distance += ps6_short_Right.getDistance();
  samplesize ++ ;
  if(samplesize == 1000){
    result = distance/1000;
    samplesize = 0;

    /* Above 25cm is not reliable
     * Only showed a new range when using an obstacle with a much bigger surface area
     * Does not to return a solid range of values, might be due to the way the sensor is installed
    */
    
    //25cm -> avg = 20.7 to 21.6, high freg of 21.5, offset = +4
    if (result >= 20){
      offset = 4;
    }
    //22.5cm -> avg = 19.49 to 19.65, offset = +3
    //21cm -> avg = 19 to 19.32, offset = +2
    else if (result >= 19 && result < 20){
      offset = 2.5;
    }
    //20cm -> avg = 18.5 to 18.8, offset = +1
    else if (result >= 18 && result < 19){
      offset = 1;
    }
    //15cm -> avg = 15.48 to 15.53
    else if (result < 18){
      offset = 0;
    }

    Serial.println(result + offset);
    distance = 0;
  }  
}

void ps3_SL(){
  distance += ps3_short_Left.getDistance();
  samplesize ++ ;
  if(samplesize == 1000){
    result = distance/1000;
    samplesize = 0;
    
    //50cm -> avg = 38.3 to 38.7, offset = +11
    //45cm -> avg = 34.72 to 36.31, offset = +10
    //42.5cm -> avg = 32.8 to 33.2
    
    //Not reliable from 40cm onwards,
    //40cm -> avg = 31.96 to 33.44, high freq of 32 and 33, offset = +6
    //37cm -> avg = 30.77 to 30.81, offset = +6
    if(result >= 30.5){
      offset = 6;
    }
    //35cm -> avg = 29.7 to 29.89, offset = +5
    else if (result >= 29.5 && result < 30.5){
      offset = 5;
    }
    //33.5cm -> avg = 29.17 to 29.20, offset =+4
    //32.5cm -> avg = 28.46 to 28.51, offst = +4
    //30cm -> avg = 25 to 26.1, offset = +4
    else if (result >= 26 && result < 29.5){
      offset = 4;
    }
    //28cm -> avg = 24.34 to 25.2, offset = +3
    //Need to use wide obstacle
    else if (result >= 23 && result < 26){
      offset = 2.5;
    }
    //25cm -> avg = 22.2 to 22.77, offset = +2
    else if (result >= 21 && result < 23){
      offset = 2;
    }
    //20cm -> avg = 20.38 to 20.57, offset = 0
    else if (result < 21){
      offset = 0;
    }

    Serial.println(result + offset);
    distance = 0;
  }
}

void ps2_SBL(){
  distance +=  ps2_short_Back_Left.getDistance();
  samplesize ++ ;
  if(samplesize == 1000){
    result = distance/1000;
    samplesize = 0;

    //40cm -> avg = 44 to 48, offset = -6
    //35cm -> avg = 40 to 42, offset = -6, **Need to use wider obstacle and not reliable from here
    //30cm -> avg = 35.45 to 36.10, offset = -6

    //29cm -> avg = 34.99 to 35.60, offset = -6 
    //28cm -> avg = 34.49 to 35.09, offset = -7
    //Need to use wide obstacle

    //27cm -> avg = 33.19 to 33.71, offset = -7
    //26cm -> avg = 31.95 to 32.22, offset = -6 
    //25cm -> avg = 31.47 to 31.75, offset = -7

    //20cm -> avg = 25.43 to 25.55, offset = -6

    offset = -6;
    Serial.println(result + offset);
    distance = 0;
  }  
}
