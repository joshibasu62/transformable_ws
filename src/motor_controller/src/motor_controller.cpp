#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>

// Motor command publishers
ros::Publisher motor1_pos_pub, motor2_pos_pub, motor3_pos_pub, motor4_pos_pub;
ros::Publisher motor1_vel_pub, motor2_vel_pub, motor3_vel_pub, motor4_vel_pub;

// Position and velocity variables
double position[4] = {0.0, 0.0, 0.0, 0.0};
double velocity[4] = {0.0, 0.0, 0.0, 0.0};

// Constants
const double POSITION_LIMIT = 1.0;  // Position limit: 0 to 1 radian
const double VELOCITY_LIMIT = 2.0;  // Velocity limit
const double POSITION_INCREMENT = 0.2;  // Increment for position changes

// Pairing mode: true for paired control, false for independent control
bool pairing_mode = false;

// Function to clamp values within limits
double clamp(double value, double min_val, double max_val) {
    return std::max(min_val, std::min(value, max_val));
}

// Function to publish motor commands
void publishCommands() {
    std_msgs::Float64 motor_msg;

    // Publish position commands
    for (int i = 0; i < 4; ++i) {
        motor_msg.data = position[i];
        switch (i) {
            case 0:
                motor1_pos_pub.publish(motor_msg);
                break;
            case 1:
                motor2_pos_pub.publish(motor_msg);
                break;
            case 2:
                motor3_pos_pub.publish(motor_msg);
                break;
            case 3:
                motor4_pos_pub.publish(motor_msg);
                break;
        }
    }

    // Publish velocity commands
    for (int i = 0; i < 4; ++i) {
        motor_msg.data = velocity[i];
        switch (i) {
            case 0:
                motor1_vel_pub.publish(motor_msg);
                break;
            case 1:
                motor2_vel_pub.publish(motor_msg);
                break;
            case 2:
                motor3_vel_pub.publish(motor_msg);
                break;
            case 3:
                motor4_vel_pub.publish(motor_msg);
                break;
        }
    }
}

// Callback for joystick control
void joystickCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    // Toggle pairing mode using a specific button (e.g., button 2 for toggle)
    if (joy->buttons[2]) {  // Button 2 toggles pairing mode
        pairing_mode = !pairing_mode;
        ROS_INFO_STREAM("Pairing mode " << (pairing_mode ? "enabled" : "disabled"));
    }

    // Handle paired control
    if (pairing_mode) {
        // Paired control: 2 motors share the same velocity/position
        velocity[0] = velocity[1] = clamp(joy->axes[1] * VELOCITY_LIMIT, -VELOCITY_LIMIT, VELOCITY_LIMIT);  // Left stick vertical
        velocity[2] = velocity[3] = clamp(joy->axes[3] * VELOCITY_LIMIT, -VELOCITY_LIMIT, VELOCITY_LIMIT);  // Right stick vertical

        if (joy->buttons[0]) {  // A button increases position
            double increment = POSITION_INCREMENT;
            position[0] = position[1] = clamp(position[0] + increment, 0.0, POSITION_LIMIT);
            position[2] = position[3] = clamp(position[2] + increment, 0.0, POSITION_LIMIT);
        }
        if (joy->buttons[1]) {  // B button decreases position
            double decrement = -POSITION_INCREMENT;
            position[0] = position[1] = clamp(position[0] + decrement, 0.0, POSITION_LIMIT);
            position[2] = position[3] = clamp(position[2] + decrement, 0.0, POSITION_LIMIT);
        }
    } else {
        // Independent control: each motor controlled separately
        velocity[0] = clamp(joy->axes[0] * VELOCITY_LIMIT, -VELOCITY_LIMIT, VELOCITY_LIMIT);  // Left stick horizontal
        velocity[1] = clamp(joy->axes[1] * VELOCITY_LIMIT, -VELOCITY_LIMIT, VELOCITY_LIMIT);  // Left stick vertical
        velocity[2] = clamp(joy->axes[2] * VELOCITY_LIMIT, -VELOCITY_LIMIT, VELOCITY_LIMIT);  // Right stick horizontal
        velocity[3] = clamp(joy->axes[3] * VELOCITY_LIMIT, -VELOCITY_LIMIT, VELOCITY_LIMIT);  // Right stick vertical

        if (joy->buttons[0]) {  // A button increases position
            for (int i = 0; i < 4; ++i) {
                position[i] = clamp(position[i] + POSITION_INCREMENT, 0.0, POSITION_LIMIT);
            }
        }
        if (joy->buttons[1]) {  // B button decreases position
            for (int i = 0; i < 4; ++i) {
                position[i] = clamp(position[i] - POSITION_INCREMENT, 0.0, POSITION_LIMIT);
            }
        }
    }

    // Publish updated commands
    publishCommands();
}
/ Pairing mode: true for paired control, false for independent control
bool pairing_mode = false;

// Function to clamp values within limits
double clamp(double value, double min_val, double max_val) {
    return std::max(min_val, std::min(value, max_val));
}

// Function to publish motor commands
void publishCommands() {
    std_msgs::Float64 motor_msg;

    // Publish position commands
    for (int i = 0; i < 4; ++i) {
        motor_msg.data = position[i];
        switch (i) {
            case 0:
                motor1_pos_pub.publish(motor_msg);
                break;
            case 1:
                motor2_pos_pub.publish(motor_msg);
                break;
            case 2:
                motor3_pos_pub.publish(motor_msg);
                break;
            case 3:
                motor4_pos_pub.publish(motor_msg);
                break;
        }
    }

    // Publish velocity commands
    for (int i = 0; i < 4; ++i) {
        motor_msg.data = velocity[i];
        switch (i) {
            case 0:
                motor1_vel_pub.publish(motor_msg);
                break;
            case 1:
                motor2_vel_pub.publish(motor_msg);
                break;
            case 2:
                motor3_vel_pub.publish(motor_msg);
                break;
            case 3:
                motor4_vel_pub.publish(motor_msg);
                break;
        }
    }
}

// Callback for joystick control
void joystickCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    // Toggle pairing mode using a specific button (e.g., button 2 for toggle)
    if (joy->buttons[2]) {  // Button 2 toggles pairing mode
        pairing_mode = !pairing_mode;
        ROS_INFO_STREAM("Pairing mode " << (pairing_mode ? "enabled" : "disabled"));
    }

    // Handle paired control
    if (pairing_mode) {
        // Paired control: 2 motors share the same velocity/position
        velocity[0] = velocity[1] = clamp(joy->axes[1] * VELOCITY_LIMIT, -VELOCITY_LIMIT, VELOCITY_LIMIT);  // Left stick vertical
        velocity[2] = velocity[3] = clamp(joy->axes[3] * VELOCITY_LIMIT, -VELOCITY_LIMIT, VELOCITY_LIMIT);  // Right stick vertical

        if (joy->buttons[0]) {  // A button increases position
            double increment = POSITION_INCREMENT;
            position[0] = position[1] = clamp(position[0] + increment, 0.0, POSITION_LIMIT);
            position[2] = position[3] = clamp(position[2] + increment, 0.0, POSITION_LIMIT);
        }
        if (joy->buttons[1]) {  // B button decreases position
            double decrement = -POSITION_INCREMENT;
            position[0] = position[1] = clamp(position[0] + decrement, 0.0, POSITION_LIMIT);
            position[2] = position[3] = clamp(position[2] + decrement, 0.0, POSITION_LIMIT);
        }
    } else {
        // Independent control: each motor controlled separately
        velocity[0] = clamp(joy->axes[0] * VELOCITY_LIMIT, -VELOCITY_LIMIT, VELOCITY_LIMIT);  // Left stick horizontal
        velocity[1] = clamp(joy->axes[1] * VELOCITY_LIMIT, -VELOCITY_LIMIT, VELOCITY_LIMIT);  // Left stick vertical
        velocity[2] = clamp(joy->axes[2] * VELOCITY_LIMIT, -VELOCITY_LIMIT, VELOCITY_LIMIT);  // Right stick horizontal
        velocity[3] = clamp(joy->axes[3] * VELOCITY_LIMIT, -VELOCITY_LIMIT, VELOCITY_LIMIT);  // Right stick vertical

        if (joy->buttons[0]) {  // A button increases position
            for (int i = 0; i < 4; ++i) {
                position[i] = clamp(position[i] + POSITION_INCREMENT, 0.0, POSITION_LIMIT);
            }
        }
        if (joy->buttons[1]) {  // B button decreases position
            for (int i = 0; i < 4; ++i) {
                position[i] = clamp(position[i] - POSITION_INCREMENT, 0.0, POSITION_LIMIT);
            }
        }
    }

    // Publish updated commands
    publishCommands();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "joystick_motor_controller");
    ros::NodeHandle nh;

    // Initialize publishers for each motor
    // motor1_pos_pub = nh.advertise<std_msgs::Float64>("/joint_1_controller/command", 10);
    // motor2_pos_pub = nh.advertise<std_msgs::Float64>("/joint_3_controller/command", 10);
    // motor3_pos_pub = nh.advertise<std_msgs::Float64>("/joint_5_controller/command", 10);
    // motor4_pos_pub = nh.advertise<std_msgs::Float64>("/joint_7_cont
    // motor1_pos_pub = nh.advertise<std_msgs::Float64>("/joint_1_controller/command", 10);
    // motor2_pos_pub = nh.advertise<std_msgs::Float64>("/joint_3_controller/command", 10);
    // motor3_pos_pub = nh.advertise<std_msgs::Float64>("/joint_5_controller/command", 10);
    // motor4_pos_pub = nh.advertise<std_msgs::Float64>("/joint_7_controller/command", 10);

    // motor1_vel_pub = nh.advertise<std_msgs::Float64>("/joint_2_controller/command", 10);
    // motor2_vel_pub = nh.advertise<std_msgs::Float64>("/joint_4_controller/command", 10);
    // motor3_vel_pub = nh.advertise<std_msgs::Float64>("/joint_6_controller/command", 10);
    // motor4_vel_pub = nh.advertise<std_msgs::Float64>("/joint_8_controller/command", 10);

    // // Subscriber for joystick inputs
    // ros::Subscriber joystick_sub = nh.subscribe("/joy", 10, joystickCallback);

    ros::spin();
    return 0;
}
