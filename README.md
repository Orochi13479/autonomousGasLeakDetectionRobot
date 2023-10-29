# Autonomous Gas Leak Detection Robot

Project using Turtlebot3 Waffle designed  search an environment, to locate the potositions of gas leaks. The turtlebot3 uses an occupacy grid map previously generated as well as AMCL for localisation. The gas leaks are emulated via the positioning of an arbitrary model within the environment, using illumience to get a "gas concetration".

First ensure required packages are installed or the next step <b>WILL FAIL</b> . These packages are written by other people and our project only utilises them and does not claim anypart of them as our own. 

<em><b>Required Packages</b></em>:
- [turtlebot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/features/)
- [ros_autonomous_slam](https://github.com/fazildgr8/ros_autonomous_slam)


To install custom packages written by @Orochi13479, Copy catkin_ws Directory on top of your current catkin_ws and replace files if prompted this should place the required files in the correct Directories. If this fails maunal drop and place the files into their corresponding directories.

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

Spawn in Cricket balls to simulate gas positions, these can be found in the default gazebo model directory or it can also be located from this [Git repository](https://github.com/osrf/gazebo_models)

```
rosrun PseudoGasSensor PseudoGasSensor 
```
```
rosrun autonomous_navigation autonomous_navigation 
```
