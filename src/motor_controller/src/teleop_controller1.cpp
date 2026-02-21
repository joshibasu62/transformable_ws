#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <termios.h>
#include <unistd.h>

// Motor command publishers
ros::Publisher motor1_pos_pub, motor2_pos_pub, motor3_pos_pub, motor4_pos_pub;
ros::Publisher motor1_vel_pub, motor2_vel_pub, motor3_vel_pub, motor4_vel_pub;
// Torque sensor publishers
ros::Publisher torque1_pub, torque2_pub, torque3_pub, torque4_pub;

// Position and velocity variables
double velocity = 0.0; // Shared velocity for all velocity motors
double position = 0.0; // Shared position for all position motors
const double POSITION_LIMIT = 1.0; // Position limit: 0 to 1 radian
const double VELOCITY_LIMIT = 10.0; // Velocity limit

// Torque variable
double torque = 0.0; // Holds the most recent torque value

// Function to clamp values within limits
double clamp(double value, double min_val, double max_val) {
return std::max(min_val, std::min(value, max_val));
}

// Function to publish velocity commands
void publishVelocity() {
std_msgs::Float64 motor_msg;
motor_msg.data = velocity;

// Publish the same velocity to all velocity motors
motor1_vel_pub.publish(motor_msg);
motor2_vel_pub.publish(motor_msg);
motor3_vel_pub.publish(motor_msg);
motor4_vel_pub.publish(motor_msg);
}

// Function to publish position commands
void publishPosition() {
std_msgs::Float64 motor_msg;
motor_msg.data = position;

// Publish the same position to all position motors
motor1_pos_pub.publish(motor_msg);
motor2_pos_pub.publish(motor_msg);
motor3_pos_pub.publish(motor_msg);
motor4_pos_pub.publish(motor_msg);
}

// Function to initialize torque publishers
void initializeTorquePublishers(ros::NodeHandle& nh) {
torque1_pub = nh.advertise<std_msgs::Float64>("/tryd/joint_1_controller/torque", 10);
torque2_pub = nh.advertise<std_msgs::Float64>("/tryd/joint_3_controller/torque", 10);
torque3_pub = nh.advertise<std_msgs::Float64>("/tryd/joint_5_controller/torque", 10);
torque4_pub = nh.advertise<std_msgs::Float64>("/tryd/joint_7_controller/torque", 10);
}

// Function to get a single character from the keyboard (non-blocking)
char getKeyPress() {
struct termios oldt, newt;
char ch;
tcgetattr(STDIN_FILENO, &oldt);
newt = oldt;
newt.c_lflag &= ~(ICANON | ECHO); // Disable buffering and echoing
tcsetattr(STDIN_FILENO, TCSANOW, &newt);
ch = getchar();
tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // Restore terminal settings
return ch;
}

// Callback for keyboard control (strictly for position control)
void keyboardCallback(const geometry_msgs::Twist::ConstPtr& msg) {
// Linear.x controls velocity (not position)
velocity = clamp(msg->linear.x * VELOCITY_LIMIT, -VELOCITY_LIMIT, VELOCITY_LIMIT);

// Handle key presses for position control
char key = getKeyPress();
if (key == 'o' || key == 'O') { // O key for 0 radians (position control)
position = 0.0;
velocity = 0.0;
} else if (key == 'l' || key == 'L') { // L key for 0.5 radians (position control)
position = 0.5;
velocity = 0.0;
} else if (key == '.') { // . key for 1 radian (position control)
position = POSITION_LIMIT;
velocity = 0.0;
}

// Publish the updated position and velocity
publishPosition();
publishVelocity();

// Debugging info
ROS_INFO("Position: %.2f, Velocity: %.2f", position, velocity);
}

// Callback for torque data
void torqueCallback(const std_msgs::Float64::ConstPtr& msg) {
torque = msg->data;
ROS_INFO("Received torque: %.2f", torque);

// Optional: Add logic to modify motor commands based on torque if needed
}

int main(int argc, char** argv) {
ros::init(argc, argv, "teleop_controller");
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
// Initialize torque publishers
initializeTorquePublishers(nh);
ROS_INFO("teleop_controller node has started.");

// Subscribe to /cmd_vel (teleop_twist_keyboard output)
ros::Subscriber keyboard_sub = nh.subscribe("/cmd_vel", 10, keyboardCallback);

// Subscribe to the torque topic
ros::Subscriber torque_sub = nh.subscribe("/motor_torque", 10, torqueCallback);

// Main loop to handle user input from keyboard
while (ros::ok()) {
ros::spinOnce(); // Continuously check for messages
}

return 0;
}
