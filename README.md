# CE3004---Mdp-Group-14-Arduino
A repo for hardware development of MDP group 14 project AY1920 S1. This documents are free for use by juniors to aid them in MDP process. Included below are hardware information, software library used, some control theory and tips and tricks on acing MDP.

1. Hardware Components:
Rough BOM for MDP (as provided by the school):

2. Software Library:
The following library are used in this project source code:
- DualVNH5019MotorShield.h : 
		This library is provided by the manufacturer of the motor shield controlling the motors. Users can give an int from -400 to 400 as PWM input for the motor turning. There are also functions to brake the motor. More details at this <a href ="https://github.com/pololu/dual-vnh5019-motor-shield">link</a>
- SharpIR.h :
		This library is to interface with Sharp IR sensors used in this project. Students are given 2 types of sensors: GP2Y0A21YK0F (10-50 cm effective range) and GP2Y0A02YK0F(20-150 cm) effective range. Some people prefers offering the characteristic equation of their sensors by themselves, do refer below for the steps to do so. Otherwise using this library will do the job. 
		Note: if less than 10 and 20cm, 9 and 19 will be returned. Details at this <a href="https://github.com/qub1750ul/Arduino_SharpIR">link</a>

3. Control Theory:

4. Tips and Tricks:
4.1. Clearing Checklist
4.2. Working towards first Exploration
4.3. How to move on

5. Link to relevant Repo
