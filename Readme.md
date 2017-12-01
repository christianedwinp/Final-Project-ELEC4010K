# HKUST ELEC 4010K FALL 2017 FINAL PROJECT
## INTRODUCTION
> This ROS package is to launch a robot in RVIZ simulation program that can do :
- Create 2D map from laser scanner
- Move within simulated environment using keyboard input
- Conduct image recognition and estimate position of the detected image
- Track yellow colour object

## PREREQUISITE
1. ROS environment (this repo refers to indigo version)
2. VREP simulator
3. RVIZ package
4. OpenCV packages

## HOW TO DEMO
>run the following task in sequential order :
1. open terminal and run `roscore`
2. open Vrep simulator and load `~/demo_elec4010k/src/env.ttt`
3. open another terminal and run `roslaunch demo_elec4010k startdemo.launch`

><p>you can navigate through a simulated environment and display 2D map in the RVIZ.
save the 2D map if needed.