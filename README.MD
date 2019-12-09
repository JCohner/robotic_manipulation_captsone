# Roboitc Manipulation Capstone Project
The capstone project for Northwestern Universities Robotic Manipulation course inolves controlling a 5 DOF robotic arm mounted on a mobile robot such that it manipulates a cube from and to specified positions. 

## Outline
The three major components of this project are:
* trajectory generation - *TrajectoryGen.py*
* feedback controll - *FeedbackControl.py*
* calculating the next configuration state - *NextState.py*

All of these are tied together in the *MobileManupulator.py* which is called as the main program of this project. The python implementation I chose to go with utlizes the modern robotics library alongside numpy, pandas, and matplotlib. 

Nothing all too surprising happened, but the degree to which the effeciency that my PI controller hinged on having a rather large degree of initial error was unexpected. To try stay away from singularities I considered any value being calculated in my jacobian inverse calculate < 1e-4 to be zero. 

## Approach
Trajctory generator served to generate a reference trajectory for the end effector relative to the cube initial and final positions. 

Feeback control takes the current configuration of our robot (13 variables) and determines the needed twist to provide based on Kp and Ki constants. In future implementations I would like to implement a Kd to improve transient response because that is lacking in this itteration.

Next state takes the current configuration and the requested twists and updates configuration variables for the next time step using body dynamics. 

All of these functional calls are implemented together to produce a csv that can be read by the V-Rep simulator using the Scene 6 of the youBot provided by the modern robotics library.  

## Implementation
All code can be found in the *cdde* directory and can be ran there with the command `$python3 MobileManupulator.py` 

