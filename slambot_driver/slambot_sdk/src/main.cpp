// src/main.cpp

#include "encoder_motor.h"
#include <iostream>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "encoder_motor_node");
    ros::NodeHandle nh;

    // Initialize the motor controller on I2C port 1
    int i2c_port = 1;
    EncoderMotorController motor_controller(i2c_port);

    // Set speed for motor ID 1
    int motor_id = 1;
    int speed = 50; // Desired speed (-100 to 100)

    try
    {
        motor_controller.setSpeed(speed, motor_id);
        std::cout << "Motor " << motor_id << " set to speed " << speed << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error setting motor speed: " << e.what() << std::endl;
    }

    // If you need to read encoder values
    try
    {
        int encoder_value = motor_controller.readEncoder(motor_id);
        std::cout << "Encoder value for motor " << motor_id << ": " << encoder_value << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error reading encoder value: " << e.what() << std::endl;
    }

    ros::spin();

    return 0;
}
