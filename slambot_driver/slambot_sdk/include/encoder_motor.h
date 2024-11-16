#ifndef ENCODER_MOTOR_H
#define ENCODER_MOTOR_H

#include <vector>
#include <stdexcept>

class EncoderMotorController
{
public:
    EncoderMotorController(int i2c_port, int motor_type = 3);
    ~EncoderMotorController();

    void setSpeed(const std::vector<int> &speed, int offset = 0);
    void setSpeed(int speed, int motor_id);
    void clearEncoder(int motor_id = -1);
    int readEncoder(int motor_id);
    std::vector<int> readAllEncoder();

private:
    int i2c_port;
    int i2c_fd;
    const int ENCODER_MOTOR_MODULE_ADDRESS = 0x34;

    void openBus();
    void closeBus();
    void writeI2CBlockData(int reg, const std::vector<uint8_t> &data);
    std::vector<uint8_t> readI2CBlockData(int reg, int length);
};

#endif
