#ifndef ICL_STEPPER_HPP
#define ICL_STEPPER_HPP

#include "arm_motor_controller/Motor.hpp"

namespace arm_motor_controller {

class ICLStepper : public Motor {

public:
    ICLStepper(std::shared_ptr<RS485> rs485, uint32_t id, double startingPos = 0);
    ~ICLStepper();

    int configure() override;

    int enable() override;
    int disable(bool isEmergency = false) override;

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
    uint16_t crc16(const uint8_t *msg, uint8_t len);

    /**
     * Reads a parameter from the motor. All read parameter downlink packets are 4 bytes
     * [ ADDR | FUNC | CRC ]
     *
     * @param function The function code to read from.
     * @param data A reference to data array to store result in.
     * @param len The length of the uplink packet data.
     *
     * @returns 0 if success, error code otherwise
     */
    int readRegister(uint16_t reg, uint16_t count, uint16_t *data);

    /**
     * Reads a parameter from the motor. All read parameter packets are 5 bytes
     * [ ADDR | FUNC | DATA | CRC ]
     *
     * @param function The function code to write from.
     * @param data A reference to data to write and store the response in.
     *
     * @returns 0 if success, error code otherwise
     */
    int writeRegister(uint16_t reg, uint16_t data);

    int writeMultiple(uint16_t reg, uint16_t count, const uint16_t *data);

    static constexpr int MAX_PACKET_LEN = 32;
    static constexpr uint8_t READ_FUNCTION_CODE = 0x03;
    static constexpr uint8_t WRITE_FUNCTION_CODE = 0x06;
    static constexpr uint8_t WRITE_MULTIPLE_FUNCTION_CODE = 0x10;

    static constexpr uint16_t ENCODER_REGISTER_ADDRESS_START = 0x602C; // Pr8.44
    static constexpr uint16_t PATH_REGISTER_ADDRESS_START = 0x6200;    // Pr9.0
    static constexpr uint16_t ENABLE_REGISTER_ADDRESS_START = 0x000F;  // Pr9.0

    static constexpr uint16_t PATH_MODE = 0x01; // Abs move, Overlaying, Interrupt, Position move
    static constexpr uint16_t PATH_DELAY = 0;
    static constexpr uint16_t PATH_TRIGGER = 0x10;

    static constexpr uint8_t ACCELERATION = 0x10;
    static constexpr uint32_t PULSES_PER_REV = 10000;
};

} // namespace arm_motor_controller

#endif // ICL_STEPPER_HPP