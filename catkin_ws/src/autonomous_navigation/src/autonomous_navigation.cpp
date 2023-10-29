#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <vector>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void gasConcentrationCallback(const std_msgs::Float64::ConstPtr& gas_concentration) {
    if (gas_concentration->data > 5.0) {
        ROS_WARN("High Pseudo Gas Concentration Detected: %f", gas_concentration->data);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "autonomous_navigation");

    // Create a NodeHandle
    ros::NodeHandle nh;

    // Initialize the move_base action client
    MoveBaseClient ac("move_base", true);

    // Wait for the action server to come up
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Please Make Sure you have run all the Commands in the README");
    }

    // Subscribe to the pseudo gas concentration topic
    ros::Subscriber gas_sub = nh.subscribe("/pseudo_gas", 10, gasConcentrationCallback);

    // List of goal positions
    std::vector<std::pair<double, double>> goals;
    goals.push_back(std::make_pair(-3.0, -1.5));
    goals.push_back(std::make_pair(-3.5, 0.0));
    goals.push_back(std::make_pair(-2.0, 1.0));
    goals.push_back(std::make_pair(3.25, -2.0));
    goals.push_back(std::make_pair(4.0, 2.75));
    goals.push_back(std::make_pair(4.0, 4.5));
    goals.push_back(std::make_pair(0.0, 4.5));
    goals.push_back(std::make_pair(-2.5, 4.5));
    goals.push_back(std::make_pair(2.5, -3.25));

    // Move to all goals in order
    for (const auto& goal : goals)
    {
        move_base_msgs::MoveBaseGoal goal_msg;
        goal_msg.target_pose.header.frame_id = "map";
        goal_msg.target_pose.header.stamp = ros::Time::now();
        goal_msg.target_pose.pose.position.x = goal.first;
        goal_msg.target_pose.pose.position.y = goal.second;
        goal_msg.target_pose.pose.orientation.w = 1.0;

        // Send the goal
        ac.sendGoal(goal_msg);
        ROS_INFO("Searching for Gas Leaks...");

        // Wait for the result
        ac.waitForResult();

        if (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("A Navigation Issue Has Occurred");
        }
        
    }
    ROS_INFO("Search for Gas Leaks is Complete.");

    ros::spin(); // Keep spinning to monitor gas concentration warnings

    return 0;
}
