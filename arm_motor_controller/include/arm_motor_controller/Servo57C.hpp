#ifndef SERVO57C_HPP
#define SERVO57C_HPP

#include "arm_motor_controller/Motor.hpp"

namespace arm_motor_controller {

class Servo57C : public Motor {

public:
    Servo57C(std::shared_ptr<RS485> rs485, uint32_t id, double startingPos = 0);
    ~Servo57C();

    int read(double time, double period);
    int write(double time, double period);

private:
    /**
     * Calculates the 8-bit CRC checksum for a packet of data
     *
     * @param msg The array of the message to calculate the CRC for.
     * @param len The length of the message.
     *
     * @returns The 8-bit CRC
     */
    uint8_t crc8(uint8_t *msg, uint8_t len);

    /**
     * Reads a parameter from the motor. All read parameter downlink packets are 4 bytes
     * [ HEAD | ADDR | FUNC | CRC ]
     *
     * @param function The function code to read from.
     * @param data A reference to data array to store result in.
     * @param len The length of the uplink packet data.
     *
     * @returns 0 if success, error code otherwise
     */
    int readParameter(uint8_t func, uint8_t *data, uint8_t dataLen);

    /**
     * Reads a parameter from the motor. All read parameter packets are 5 bytes
     * [ HEAD | ADDR | FUNC | DATA | CRC ]
     *
     * @param function The function code to write from.
     * @param data A reference to data to write and store the response in.
     *
     * @returns 0 if success, error code otherwise
     */
    int writeParameter(uint8_t func, uint8_t &data);

    int sendCommand(uint8_t func, uint8_t &data, uint8_t len, uint8_t responseLen);

    static constexpr int MAX_PACKET_LEN = 11;
    static constexpr uint8_t DOWNLINK_PACKET_HEAD = 0xFA;
    static constexpr uint8_t UPLINK_PACKET_HEAD = 0xFB;

    static constexpr uint8_t ACCELERATION = 30;
    static constexpr uint16_t ENCODER_VALUE_MAX = 0x4000;
    static constexpr uint32_t PULSES_PER_REV = 3200;

    // int16_t previousVel;
    // int32_t prevPos;
};

} // namespace arm_motor_controller

#endif // SERVO57C_HPP