#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>

// Motor command publishers
ros::Publisher motor1_pos_pub, motor2_pos_pub, motor3_pos_pub, motor4_pos_pub;
ros::Publisher motor1_vel_pub, motor2_vel_pub, motor3_vel_pub, motor4_vel_pub;

// Position and velocity variables
double velocity = 0.0;
double position = 0.0;
const double POSITION_LIMIT = 1.0;  // Position limit: 0 to 1 radian
const double VELOCITY_LIMIT = 10.0; // Velocity limit

// Joystick button mappings (modify based on Logitech joystick layout)
const int BUTTON_O = 0; // Example: Button A
const int BUTTON_L = 1; // Example: Button B
const int BUTTON_DOT = 2; // Example: Button X
const int AXIS_VELOCITY = 1; // Example: Left joystick Y-axis

// Function to clamp values within limits
double clamp(double value, double min_val, double max_val) {
    return std::max(min_val, std::min(value, max_val));
}

// Function to publish velocity commands
void publishVelocity() {
    std_msgs::Float64 motor_msg;
    motor_msg.data = velocity;

    motor1_vel_pub.publish(motor_msg);
    motor2_vel_pub.publish(motor_msg);
    motor3_vel_pub.publish(motor_msg);
    motor4_vel_pub.publish(motor_msg);
}

// Function to publish position commands
void publishPosition() {
    std_msgs::Float64 motor_msg;
    motor_msg.data = position;

    motor1_pos_pub.publish(motor_msg);
    motor2_pos_pub.publish(motor_msg);
    motor3_pos_pub.publish(motor_msg);
    motor4_pos_pub.publish(motor_msg);
}

// Joystick callback function
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    // Button mappings for position control
    if (joy->buttons[BUTTON_O]) {  // Button A
        position = 0.0;
        velocity = 0.0;
    } else if (joy->buttons[BUTTON_L]) {  // Button B
        position = 0.5;
        velocity = 0.0;
    } else if (joy->buttons[BUTTON_DOT]) {  // Button X
        position = POSITION_LIMIT;
        velocity = 0.0;
    }

    // Axis mapping for velocity control (left joystick Y-axis)
    velocity = clamp(joy->axes[AXIS_VELOCITY] * VELOCITY_LIMIT, -VELOCITY_LIMIT, VELOCITY_LIMIT);

    // Publish the updated position and velocity
    publishPosition();
    publishVelocity();

    // Debugging info
    ROS_INFO("Joystick Input - Position: %.2f, Velocity: %.2f", position, velocity);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "joystick_controller");
    ros::NodeHandle nh;

    // Initialize publishers for each motor
    motor1_vel_pub = nh.advertise<std_msgs::Float64>("/tryd/joint_2_controller/command", 10);
    motor2_vel_pub = nh.advertise<std_msgs::Float64>("/tryd/joint_4_controller/command", 10);
    motor3_vel_pub = nh.advertise<std_msgs::Float64>("/tryd/joint_6_controller/command", 10);
    motor4_vel_pub = nh.advertise<std_msgs::Float64>("/tryd/joint_8_controller/command", 10);

    motor1_pos_pub = nh.advertise<std_msgs::Float64>("/tryd/joint_1_controller/command", 10);
    motor2_pos_pub = nh.advertise<std_msgs::Float64>("/tryd/joint_3_controller/command", 10);
    motor3_pos_pub = nh.advertise<std_msgs::Float64>("/tryd/joint_5_controller/command", 10);
    motor4_pos_pub = nh.advertise<std_msgs::Float64>("/tryd/joint_7_controller/command", 10);

    // Subscribe to joystick messages
    ros::Subscriber joy_sub = nh.subscribe("/joy", 10, joyCallback);

    ROS_INFO("Joystick teleop controller started. Use joystick buttons for position and axes for velocity control.");

    // ROS event loop
    ros::spin();

    return 0;
}
