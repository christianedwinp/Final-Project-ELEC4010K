#INTRODUCTION
This ROS package is to launch a robot in RVIZ simulation program that can do :
1. Create 2D map from laser scanner
2. Move within simulated environment using keyboard input
3. Conduct image recognition and estimate position of the detected image
4. Track yellow colour object

#PREREQUISITE
1. ROS distro, in this case we use indigo
2. VREP simulator
3. rviz packages
4. opencv packages

#HOW TO DEMO
run the following task in sequential order :
1. open terminal and run 'roscore'
2. open Vrep simulator and load '~/demo_elec4010k/src/env.ttt'
3. open another terminal and run 'roslaunch demo_elec4010k startdemo.launch'

you can navigate through a simulated environment and display 2D map in the RVIZ.
save the 2D map if needed.