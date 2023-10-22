# autonomousGasLeakDetectionRobot


First ensure required packages are installed or the next step <b>WILL FAIL</b> .

<em><b>Required Packages</b></em>:
package1
package2
....


Copy catkin_ws Directory on top of your current catkin_ws and replace files if prompted this should place the required files in the correct Directories. If this fails maunal do it


run catkin_make


roslaunch turtlebot3_gazebo turtlebot3_office.launch

roslaunch ros_autonomous_slam turtlebot3_navigation.launch 
(Disclaimer: run twice if doesn't work first time, relaunch again)

rosrun autonomous_navigation autonomous_navigation 
