#include "encoder_motor.h"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstring>

EncoderMotorController::EncoderMotorController(int i2c_port, int motor_type)
    : i2c_port(i2c_port), i2c_fd(-1)
{
    openBus();
    // Initialize the motor controller with the motor type
    std::vector<uint8_t> data = {static_cast<uint8_t>(motor_type)};
    writeI2CBlockData(20, data);
}

EncoderMotorController::~EncoderMotorController()
{
    closeBus();
}

void EncoderMotorController::openBus()
{
    std::string filename = "/dev/i2c-" + std::to_string(i2c_port);
    i2c_fd = open(filename.c_str(), O_RDWR);
    if (i2c_fd < 0)
    {
        throw std::runtime_error("Failed to open the I2C bus");
    }
    if (ioctl(i2c_fd, I2C_SLAVE, ENCODER_MOTOR_MODULE_ADDRESS) < 0)
    {
        throw std::runtime_error("Failed to acquire bus access and/or talk to slave");
    }
}

void EncoderMotorController::closeBus()
{
    if (i2c_fd >= 0)
    {
        close(i2c_fd);
        i2c_fd = -1;
    }
}

void EncoderMotorController::writeI2CBlockData(int reg, const std::vector<uint8_t> &data)
{
    uint8_t buffer[1 + data.size()];
    buffer[0] = static_cast<uint8_t>(reg);
    memcpy(buffer + 1, data.data(), data.size());

    ssize_t result = write(i2c_fd, buffer, sizeof(buffer));
    if (result != static_cast<ssize_t>(sizeof(buffer)))
    {
        throw std::runtime_error("Failed to write to the I2C bus");
    }
}

std::vector<uint8_t> EncoderMotorController::readI2CBlockData(int reg, int length)
{
    uint8_t reg_buf[1] = {static_cast<uint8_t>(reg)};
    if (write(i2c_fd, reg_buf, 1) != 1)
    {
        throw std::runtime_error("Failed to write to the I2C bus");
    }
    std::vector<uint8_t> data(length);
    if (read(i2c_fd, data.data(), length) != length)
    {
        throw std::runtime_error("Failed to read from the I2C bus");
    }
    return data;
}

void EncoderMotorController::setSpeed(const std::vector<int> &speed, int offset)
{
    for (size_t id_index = 0; id_index < speed.size(); ++id_index)
    {
        int sp = speed[id_index];
        int motor_id = id_index + 1;
        sp = std::max(-100, std::min(100, sp));
        try
        {
            writeI2CBlockData(50 + motor_id, {static_cast<uint8_t>(sp)});
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << std::endl;
            writeI2CBlockData(50 + motor_id, {static_cast<uint8_t>(sp)});
        }
    }
}

void EncoderMotorController::setSpeed(int speed, int motor_id)
{
    if (motor_id < 1 || motor_id > 3)
    {
        throw std::invalid_argument("Invalid motor id");
    }

    speed = std::max(-100, std::min(100, speed));

    try
    {
        writeI2CBlockData(50 + motor_id, {static_cast<uint8_t>(speed)});
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        writeI2CBlockData(50 + motor_id, {static_cast<uint8_t>(speed)});
    }
}

void EncoderMotorController::clearEncoder(int motor_id)
{
    if (motor_id == -1)
    {
        // Clear all encoders
        std::vector<uint8_t> zeros(16, 0);
        writeI2CBlockData(60, zeros);
    }
    else
    {
        if (motor_id < 1 || motor_id > 3)
        {
            throw std::invalid_argument("Invalid motor id");
        }
        std::vector<uint8_t> zeros(4, 0);
        writeI2CBlockData(60 + motor_id * 4, zeros);
    }
}

int EncoderMotorController::readEncoder(int motor_id)
{
    if (motor_id < 1 || motor_id > 3)
    {
        throw std::invalid_argument("Invalid motor id");
    }

    std::vector<uint8_t> data = readI2CBlockData(60 + motor_id * 4, 4);
    int32_t count = 0;
    memcpy(&count, data.data(), 4);
    return static_cast<int>(count);
}

std::vector<int> EncoderMotorController::readAllEncoder()
{
    std::vector<uint8_t> data = readI2CBlockData(60, 16);
    std::vector<int> counts(2);
    for (int i = 0; i < counts.size(); ++i)
    {
        int32_t count = 0;
        memcpy(&count, &data[i * 4], 4);
        counts[i] = static_cast<int>(count);
    }
    return counts;
}