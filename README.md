# CE3004---Mdp-Group-14-Arduino
A repo for hardware development of MDP group 14 project AY1920 S1. This documents are free for use by juniors to aid them in MDP process. Included below are hardware information, software library used, some control theory and tips and tricks on acing MDP. All SCSE students have mixed review about this mod, personally I believe it needs to be revamped!!!

## Project Overview (full-team):
This is "Ideally" a very good course as it teach students about robotics with topical tasks such as path planning, exploration, navigation, perception, mapping, etc. A project team of 8-10 are supposedly complete the following features within a semester (which is absurd giving the hardware limitations, knowledges limitations and workloads of other modules):

1. Algorithm Team - Path planning, Mapping, Localization, Navigation

The last that I know, the codebase should be in Java. Main goal of this sub-team is to generate an empty 2D occupancy map, using some navigation and graph planning algorithm and integrate with sensor data (such as IR sensors, ultrasonic sensors) to detect blocks in the maze and map them out in occupancy map (probabilistic model in robotics).

At the end of exploration, the accurate map of the mazed should be generated and the robot is to attempt a fastest path test where it runs from start to end position of the maze in as shortest time as possible.

Whoever do this part, they will have to be able to do some algorithm simulation of the robot on the computer (good for algorithmic people lol)

2. Arduino Team - Platform and APIs for robot mobility (to achieve some higher level classes)

In order for robot to be able localise itself in the maze, sensory information are important as this help to roughly gauge its pose in the predetermined maze (and so grid). The algorithm cannot work in real life if the robot cannot move and if the robot is blind (cannot sense what is around it or how fast it has been moving).

The Arduino team are often consists of CE students, who are supposed to be well versed in microcontroller programming techniques, DSP, sensors and signals, etc. This sub team will be working with 2-3 sensors below:

* Acoustic Sensors (Ultrasonic Sensors) - very low resolution and echo-ing problem in the maze -> generate absolute position information
* Infrared Rangefinders - 5 shorts range and 2 long range sensors providing, more accurate than the above one -> generate absolute position information
* Wheel encoders - for wheel odometry by interrupts to determined how much the wheel have been rotated -> generate relative position information

These sensors should be calibrated and integrated with the Arduno Uno microcontroller and passed through some signal processing such as ADC before it is reported to the Algorithm for update of the map, navigation and mobility, etc.

The team will also be responsible for mobility of the robot, making sure that the robot can rotate and go straight accordingly, and ensure good calibration of relevant components to minimise drifting of the wheels, etc.

NOTE: THIS IS TOUGH AS MOST OF US ARE NOT EVEN WELL-TRAINED IN ROBOTICS TO BEGIN WITH.

3. Rpi team - Communication, Scheduling, Image Processing:
4. Android team - Human Machine Interface

## Navigate this repo
Please following this order to understand the project best:
	
* __For Checklists__: containing codes to clear the checklist. The current codes were written by Bronson Tan and serves as a general flow and algorithms guide on how to to clear the checklist (for each category of going straight, rotate left/right, sensors check and obstacle avoidance

* __Calibration & Scripts__: The method of calibrating the encoders to motors speed by obtaining linear equation and select the suitable linear characteristic portion of the transfer graph. The calibration contains intial calibration of our encoders without loads and the Scripts contain a python code to automate the calculation of average values. Though some excel calculation is needed to finally obtain the RPM to PWM speed. Refer to details below 

# 1. Hardware Components:
Rough BOM for MDP - Robot Kit (as provided by the school):
*TODO*

# 2. Software Library:
The following library are used in this project source code:
* __DualVNH5019MotorShield.h__:
	This library is provided by the manufacturer of the motor shield controlling the motors. Users can give an int from -400 to 400 		as PWM input for the motor turning. There are also functions to brake the motor. More details at this [Link](https://github.com/pololu/dual-vnh5019-motor-shield)
* __SharpIR.h__ :
	This library is to interface with Sharp IR sensors used in this project. Students are given 2 types of sensors: GP2Y0A21YK0F (10-50 cm effective range) and GP2Y0A02YK0F(20-150 cm) effective range. Some people prefers offering the characteristic equation of their sensors by themselves, do refer below for the steps to do so. Otherwise using this library will do the job. 
	Note: if less than 10 and 20cm, 9 and 19 will be returned. Details at this [Link](https://github.com/qub1750ul/Arduino_SharpIR)
* __EnableInterrupt.h__:
	Arduino Uno only allows interrupts on limited amount of pins. This library allow interrupt event for all digital pins on the Uno board. This is important to develop ISR for ticks and encode handling.
* __PID_v1.h__:
	PID algorithm might be too time-consuming to develop in this 12 weeks project. *Readers are highly encouraged to skipped the theory of PID and use this library*. Final codes have 1 portion at the end to explain PID in details should the readers are curious about the idea behind this algorithm
* __ArduinoSort.h__:
	Just an insertion sort algorithm for implementing median filtering of sensor value. Other sorting algo will works too. But take note of the limited time between each steps in the maze and/or limited memory resource of the Arduino Board
	
## 3. Control Theory:

## 4. Tips and Tricks:
* Rule 1: Don't rely on your prof, their answers will have little contribution on your robot.
* Rule 2: Check all of your robot components. Change any components earlier on, protect your components well (all hardware are fragile and prone to low error tolerance)
* Rule 3: Chase your teammates, force them to collaborate. Many times your teammate are blur or just slacking, make sure to scare them because you can shoot them down with the peer evaluation. 
* Rule 4: Practice scientific methodology in dev and test - ensure you can replicate the operation of robot by keeping light level, surface condition and battery level as constant as possible. You will learn about how these factors affect your robot as you go along.
* Rule 5: Pray hard and try, try, try again. 

### 4.1. Clearing Checklist
There are 5 checklists for robot team: Going straight, Turn left/right, Sensor reading, Object Avoidance and Bonus Object Avoidance. As a rule of thumbs, seek to clear these by week 5, latest before recess week where the ramping up will start. 
Most of the groups will waste time doing the checklists so it is important to just clear the checklist fast and sneakily as all of them are useless to leaderboard anyway.
### Checklist 1: Going straight
KEEP THE ROBOT LIGHT!!!
Install barely the Arduino kit, the battery, the motors, that's it. The less components, the easier it is to manourve its centre of gravity. There are 2 methods you may seek to clear this one easier and one harder. Both methods will still require a close attention to battery voltage level. GET YOURSELF A MULTI-METER EARLIER ON!!!

__Method 1: Drive by PWM__
* Use the DualVNH Library to turn the motors forward. Due to errors and tolerances, the same value given (0-400) will not yield the same amount of rotation between the motors.
* Determine the two values of motor 1 and 2 (should be very close) such that both turns at the same rate and/or turn at a rate that feel like going straight for the 150cm required by the checklist.
* Replicate with constant battery conditions/ surface conditions.

__Method 2: Drive by RPM__
* Obtain a calibration chart for each motor, note the controlled voltage level (like method 1)
* For each speed (0-400) with an interval of your choice (20-50 per step is good), use the encoders and ISR to measure the ticks generated. From there calculate the actual RPM rotated by each motor at each speed. Details can be found at __Calibration__ folder.
* Make sure your battery/ surface is the same as calibrated condition, give the same rpm to both wheel (adjust the rpm if needed faster or slower) until robot can go straight.

### 4.2. Working towards first Exploration
### 4.3. How to move on

# 5. What can be done better?
Apology for very messy structure of the repository. If you are taking MDP this sem, i wish you the best for your MDP journey, not gonna easy.

Arduino code tends to be pass down from seniors to juniors, however, if you are willing to learn or make a difference in your run, here are some suggestion from me (Arduino):

- Incorporate an RTOS for Arduino, maybe check out FreeRTOS by Amazon for Arduino task management (even better perfomance than just an infinite loop - barebone right now)
- Use the ultrasonic sensor.
- Less worry about PID, more focus on recalibration (if your movement screw up, can at least recalculate)
- Don't be too hard on yourself, CE people! (Robots are meant to be smash!)

