#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <vector>

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

    // List of goal positions
    std::vector<std::pair<double, double>> goals;
    

    for (const auto& goal : goals)
    {
        move_base_msgs::MoveBaseGoal goal_msg;
        goal_msg.target_pose.header.frame_id = "map";
        goal_msg.target_pose.header.stamp = ros::Time::now();
        goal_msg.target_pose.pose.position.x = goal.first;
        goal_msg.target_pose.pose.position.y = goal.second;
        // goal_msg.target_pose.pose.orientation.w = 1.0;

        // Send the goal
        ROS_INFO("Sending goal...");
        ac.sendGoal(goal_msg);

        // Wait for the result
        ac.waitForResult();

        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Goal Reached");
        }
        else
        {
            ROS_INFO("Goal Couldn't Be Reached");
        }
    }

    return 0;
}
