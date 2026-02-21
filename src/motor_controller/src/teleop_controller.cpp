#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <unitree_legged_msgs/MotorCmd.h>
#include <algorithm>

// Motor command publishers
ros::Publisher motor1_pos_pub, motor2_pos_pub, motor3_pos_pub, motor4_pos_pub;
ros::Publisher motor1_vel_pub, motor2_vel_pub, motor3_vel_pub, motor4_vel_pub;

// Position and velocity variables
double velocity = 0.0;
double position = 0.0;
const double POSITION_LIMIT = 1.0;   // Max position [rad]
const double VELOCITY_LIMIT = 10.0;  // Max velocity [rad/s]

// Clamp helper
double clamp(double value, double min_val, double max_val) {
    return std::max(min_val, std::min(value, max_val));
}

// Publish position command
void publishPosition() {
    unitree_legged_msgs::MotorCmd motor_msg;
    motor_msg.mode = 10;  // Position control
    motor_msg.q = position;
    motor_msg.dq = 0.0;
    motor_msg.Kp = 20.0;
    motor_msg.Kd = 0.0;
    motor_msg.tau = 0.0;

    motor1_pos_pub.publish(motor_msg);
    motor2_pos_pub.publish(motor_msg);
    motor3_pos_pub.publish(motor_msg);
    motor4_pos_pub.publish(motor_msg);
}

// Publish velocity command
void publishVelocity() {
    unitree_legged_msgs::MotorCmd motor_msg;
    motor_msg.mode = 10;  // Velocity control
    motor_msg.q = 0.0;
    motor_msg.dq = velocity;
    motor_msg.Kp = 0.0;
    motor_msg.Kd = 5.0;
    motor_msg.tau = 0.0;

    motor1_vel_pub.publish(motor_msg);
    motor2_vel_pub.publish(motor_msg);
    motor3_vel_pub.publish(motor_msg);
    motor4_vel_pub.publish(motor_msg);
}

// Receive Twist command from /cmd_vel
void keyboardCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    velocity = clamp(msg->linear.x * VELOCITY_LIMIT, -VELOCITY_LIMIT, VELOCITY_LIMIT);
    position = clamp(msg->angular.z, -POSITION_LIMIT, POSITION_LIMIT);  // Use angular.z as a proxy for position setpoint

    publishPosition();
    publishVelocity();

    ROS_INFO("Received: linear.x=%.2f, angular.z=%.2f | Sent: Pos=%.2f Vel=%.2f",
             msg->linear.x, msg->angular.z, position, velocity);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "teleop_controller");
    ros::NodeHandle nh;

    // Position publishers
    motor1_pos_pub = nh.advertise<unitree_legged_msgs::MotorCmd>("/tryd/joint_1_controller/command", 10);
    motor2_pos_pub = nh.advertise<unitree_legged_msgs::MotorCmd>("/tryd/joint_3_controller/command", 10);
    motor3_pos_pub = nh.advertise<unitree_legged_msgs::MotorCmd>("/tryd/joint_5_controller/command", 10);
    motor4_pos_pub = nh.advertise<unitree_legged_msgs::MotorCmd>("/tryd/joint_7_controller/command", 10);

    // Velocity publishers
    motor1_vel_pub = nh.advertise<unitree_legged_msgs::MotorCmd>("/tryd/joint_2_controller/command", 10);
    motor2_vel_pub = nh.advertise<unitree_legged_msgs::MotorCmd>("/tryd/joint_4_controller/command", 10);
    motor3_vel_pub = nh.advertise<unitree_legged_msgs::MotorCmd>("/tryd/joint_6_controller/command", 10);
    motor4_vel_pub = nh.advertise<unitree_legged_msgs::MotorCmd>("/tryd/joint_8_controller/command", 10);

    ros::Subscriber keyboard_sub = nh.subscribe("/cmd_vel", 10, keyboardCallback);

    ROS_INFO("teleop_controller node running. Use teleop_twist_keyboard to publish to /cmd_vel.");
    ros::spin();

    return 0;
}
