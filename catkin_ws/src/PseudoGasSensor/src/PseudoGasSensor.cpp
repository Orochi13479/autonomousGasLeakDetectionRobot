#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/Illuminance.h>
#include <geometry_msgs/Pose.h>
#include <cmath>
#include <string>

class PseudoGasSensor {
public:
    PseudoGasSensor() {
        ros::NodeHandle nh;
        model_sub = nh.subscribe("/gazebo/model_states", 10, &PseudoGasSensor::modelStatesCallback, this);
    }

    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        int turtlebot_idx = -1;
        std::vector<int> light_indices;
        
        // Find TurtleBot3 index
        for (unsigned int i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "turtlebot3") {
                turtlebot_idx = i;
            } else if (msg->name[i].find("cricket_ball") != std::string::npos) {
                light_indices.push_back(i);
            }
        }

        if (turtlebot_idx != -1 && !light_indices.empty()) {
            float closest_distance = std::numeric_limits<float>::max();
            for (const int& light_idx : light_indices) {
                // Get positions of TurtleBot3 and cricket_ball
                float turtlebot_x = msg->pose[turtlebot_idx].position.x;
                float turtlebot_y = msg->pose[turtlebot_idx].position.y;
                float light_x = msg->pose[light_idx].position.x;
                float light_y = msg->pose[light_idx].position.y;

                // Calculate distance between TurtleBot3 and the cricket_ball
                float distance = sqrt(pow(turtlebot_x - light_x, 2) + pow(turtlebot_y - light_y, 2));

                // Find the closest cricket_ball to the TurtleBot3
                if (distance < closest_distance) {
                    closest_distance = distance;
                }
            }

            // Calculate gas concentration based on the closest distance
            float gas_concentration = calculateGasConcentration(closest_distance);

            // Print the gas concentration reading
            ROS_INFO("Pseudo Gas Reading: %f", gas_concentration);
        }
    }
    
    float calculateGasConcentration(float distance) {
        // Custom function for gas concentration within the specified range
        float max_distance = 2.0;  // Maximum distance
        float min_distance = 0.5;  // Minimum distance for peak concentration
        float scale_factor = 10.0; // Desired maximum concentration

        if (distance <= min_distance) {
            return scale_factor;  // Within 0.5 meters, close to maximum concentration (around 10)
        } else if (distance >= max_distance) {
            return 0.0;  // Beyond 2 meters, close to minimum concentration (around 0)
        } else {
            // Map distance inversely logarithmically within the given range
            float normalized_distance = (distance - min_distance) / (max_distance - min_distance);
            return scale_factor * (1 - normalized_distance);
        }
    }

private:
    ros::Subscriber model_sub;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pseudo_gas_sensor");
    PseudoGasSensor sensor;
    ros::spin();
    return 0;
}
