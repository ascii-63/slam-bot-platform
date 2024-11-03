// src/main.cpp

#include "encoder_motor.h"
#include <iostream>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "encoder_motor_node");
    ros::NodeHandle nh("~"); // Private NodeHandle for parameters

    // Initialize the motor controller on I2C port 1
    int i2c_port = 1;
    EncoderMotorController motor_controller(i2c_port);

    // Retrieve motor speeds from parameters
    int speed_motor1 = 0;
    int speed_motor2 = 0;

    nh.param("speed_motor1", speed_motor1, 0);
    nh.param("speed_motor2", speed_motor2, 0);

    // Ensure speeds are within valid range
    speed_motor1 = std::max(-100, std::min(100, speed_motor1));
    speed_motor2 = std::max(-100, std::min(100, speed_motor2));

    try
    {
        motor_controller.setSpeed(speed_motor1, 1); // Motor ID 1
        motor_controller.setSpeed(speed_motor2, 2); // Motor ID 2

        ROS_INFO("Set motor speeds: Motor 1 = %d, Motor 2 = %d", speed_motor1, speed_motor2);
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Error setting motor speeds: %s", e.what());
    }

    ros::spin();

    return 0;
}
