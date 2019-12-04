# CE3004---Mdp-Group-14-Arduino
<p>A repo for hardware development of MDP group 14 project AY1920 S1. This documents are free for use by juniors to aid them in MDP process. Included below are hardware information, software library used, some control theory and tips and tricks on acing MDP.</p>

<h2> Navigation Guidelines </h2>
<p> Please following this order to understand the project best</p>
<ol>
	<li> For Checklists: containing codes to clear the checklist. The current codes were written by Bronson Tan and serves as a general flow and algorithms guide on how to to clear the checklist (for each category of going straight, rotate left/right, sensors check and obstacle avoidance</li>
	<li> Calibration & Scripts: The method of calibrating the encoders to motors speed by obtaining linear equation and select the suitable linear characteristic portion of the transfer graph. The calibration contains intial calibration of our encoders without loads and the Scripts contain a python code to automate the calculation of average values. Though some excel calculation is needed to finally obtain the RPM to PWM speed. Refer to details below </li>
</ol>

<h2>1. Hardware Components:</h2>
Rough BOM for MDP (as provided by the school):

<h2>2. Software Library:</h2>
<p>The following library are used in this project source code:</p>
<ul>
	<li><b>DualVNH5019MotorShield.h:</b>
	<p>This library is provided by the manufacturer of the motor shield controlling the motors. Users can give an int from -400 to 400 		as PWM input for the motor turning. There are also functions to brake the motor. More details at this <a href 		="https://github.com/pololu/dual-vnh5019-motor-shield">link</a>
	</p>
	</li>
	<li><b>SharpIR.h :</b>
		<p>This library is to interface with Sharp IR sensors used in this project. Students are given 2 types of sensors: GP2Y0A21YK0F (10-50 cm effective range) and GP2Y0A02YK0F(20-150 cm) effective range. Some people prefers offering the characteristic equation of their sensors by themselves, do refer below for the steps to do so. Otherwise using this library will do the job. 
		Note: if less than 10 and 20cm, 9 and 19 will be returned. Details at this <a href="https://github.com/qub1750ul/Arduino_SharpIR">link</a></p>
	</li>
</ul>
<h2>3. Control Theory:</h2>

<h2>4. Tips and Tricks:</h2>
<h3>4.1. Clearing Checklist</h3>
<h3>4.2. Working towards first Exploration</h3>
<h3>4.3. How to move on</h3>


