#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>

// Global variables to store the bias (offset)
std::vector<double> force_offset(3, 0.0);
std::vector<double> torque_offset(3, 0.0);

// Callback function to process force-torque data
void ftSensorCallback(const geometry_msgs::Wrench::ConstPtr& msg)
{
    // If the offset hasn't been set yet, initialize it during the first reading
    static bool first_reading = true;

    if (first_reading)
    {
        // Initialize the offsets when the object is stationary
        force_offset[0] = msg->force.x;
        force_offset[1] = msg->force.y;
        force_offset[2] = msg->force.z;

        torque_offset[0] = msg->torque.x;
        torque_offset[1] = msg->torque.y;
        torque_offset[2] = msg->torque.z;

        ROS_INFO("Sensor calibrated, offset set.");
        first_reading = false;
        return;
    }

    // Subtract the offset from the incoming force and torque values
    double corrected_force_x = msg->force.x - force_offset[0];
    double corrected_force_y = msg->force.y - force_offset[1];
    double corrected_force_z = msg->force.z - force_offset[2];

    double corrected_torque_x = msg->torque.x - torque_offset[0];
    double corrected_torque_y = msg->torque.y - torque_offset[1];
    double corrected_torque_z = msg->torque.z - torque_offset[2];

    // Log the corrected values
    ROS_INFO("Corrected Force: x=%.2f, y=%.2f, z=%.2f", 
             corrected_force_x, corrected_force_y, corrected_force_z);
    ROS_INFO("Corrected Torque: x=%.2f, y=%.2f, z=%.2f", 
             corrected_torque_x, corrected_torque_y, corrected_torque_z);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ft_sensor_zeroing_node");
    ros::NodeHandle nh;

    // Subscribe to the force-torque sensor topic
    ros::Subscriber sub = nh.subscribe("/ft_sensor_4_topic", 1000, ftSensorCallback);

    ros::spin();

    return 0;
}
