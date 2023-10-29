# Autonomous Gas Leak Detection Robot

Project using Turtlebot3 Waffle designed to search an environment, using SLAM, to locate the positions of gas leaks. The gas leaks are emulated via the positioning of an arbitrary model within the environment, using illumience to get a "gas concetration".

First ensure required packages are installed or the next step <b>WILL FAIL</b> . These packages are written by other people and our project only utilises them and does not claim anypart of them as our own. 

<em><b>Required Packages</b></em>:
turtlebot3
ros_autonomous_slam
....


To install custom packages written by @Orochi13479 please, Copy catkin_ws Directory on top of your current catkin_ws and replace files if prompted this should place the required files in the correct Directories. If this fails maunal drop and place the files into their corresponding directories.

```
catkin_make
```
```
roslaunch turtlebot3_gazebo turtlebot3_office.launch
```
```
roslaunch ros_autonomous_slam turtlebot3_navigation.launch 
```
(Disclaimer: run twice if doesn't work first time, relaunch again)

<em><b>Custom Packages, Run in order</b></em>:

Spawn in Cricket balls to simulate gas positions, these can be found in the default gazebo model directory

```
rosrun PseudoGasSensor PseudoGasSensor 
```
```
rosrun autonomous_navigation autonomous_navigation 
```
