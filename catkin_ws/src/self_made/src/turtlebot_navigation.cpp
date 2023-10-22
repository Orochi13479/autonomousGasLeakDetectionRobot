#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtlebot3_navigation");

    // Create a NodeHandle
    ros::NodeHandle nh;

    // Initialize the move_base action client
    MoveBaseClient ac("move_base", true);

    // Wait for the action server to come up
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up...");
    }

    // Prompt the user for the start position
    double start_x, start_y;
    std::cout << "Enter start position (x y): ";
    std::cin >> start_x >> start_y;

    // Create and fill in the goal message for the starting point
    move_base_msgs::MoveBaseGoal start_goal;
    start_goal.target_pose.header.frame_id = "map";
    start_goal.target_pose.header.stamp = ros::Time::now();
    start_goal.target_pose.pose.position.x = start_x;
    start_goal.target_pose.pose.position.y = start_y;
    start_goal.target_pose.pose.orientation.w = 1.0;

    // Send the starting point goal
    ROS_INFO("Sending starting point goal...");
    ac.sendGoal(start_goal);

    // Wait for the TurtleBot to reach the starting point
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("TurtleBot has reached the starting point.");
    }
    else
    {
        ROS_INFO("TurtleBot failed to reach the starting point.");
        return 1;
    }

    // Prompt the user for the goal position
    double goal_x, goal_y;
    std::cout << "Enter goal position (x y): ";
    std::cin >> goal_x >> goal_y;

    // Create and fill in the goal message for the goal point
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = goal_x;
    goal.target_pose.pose.position.y = goal_y;
    goal.target_pose.pose.orientation.w = 1.0;

    // Send the goal
    ROS_INFO("Sending goal...");
    ac.sendGoal(goal);

    // Wait for the result
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Hooray, the base reached the goal!");
    }
    else
    {
        ROS_INFO("The base failed to reach the goal for some reason.");
    }

    return 0;
}
