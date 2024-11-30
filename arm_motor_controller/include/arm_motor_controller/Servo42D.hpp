#ifndef SERVO42D_HPP
#define SERVO42D_HPP

#include "arm_motor_controller/Communication/ModbusDevice.hpp"
#include "arm_motor_controller/Motor.hpp"

namespace arm_motor_controller {

class Servo42D : public Motor {

public:
    Servo42D(std::shared_ptr<RS485> rs485, uint32_t id, bool isReversed = false);
    ~Servo42D();

    void setMotorHome(uint32_t speed, uint32_t config) override;

    int configure() override;

    int enable() override;
    int disable(bool isEmergency = false) override;

    int read(double time, double period) override;
    int write(double time, double period) override;

private:
    /* Modbus constants */
    static constexpr double ERROR_DECAY_RATE = 0.25;
    static constexpr uint8_t READ_FUNCTION_CODE = 0x04;
    static constexpr uint8_t WRITE_FUNCTION_CODE = 0x06;
    static constexpr uint8_t WRITE_MULTIPLE_FUNCTION_CODE = 0x10;

    /* Configuration Registers */

    // static constexpr uint16_t DIGITAL_IN1_CONFIG_REGISTER_ADDRESS = 0x0145; // Pr4.02
    // static constexpr uint16_t DIGITAL_IN7_CONFIG_REGISTER_ADDRESS = 0x0151; // Pr4.08
    // static constexpr uint16_t ALARM_CONFIG_REGISTER_ADDRESS = 0x016D;       // Pr4.22
    // static constexpr uint16_t STATUS_REGISTER_ADDRESS = 0x1003;             // Pr---
    // static constexpr uint16_t CONTROL_WORD_REGISTER_ADDRESS = 0x1801;       // Pr---
    // static constexpr uint16_t PATH_CONTROL_REGISTER_ADDRESS = 0x6002;       // Pr8.02
    static constexpr uint16_t ENCODER_REGISTER_ADDRESS_START = 0x031;    // 8.1.2
    static constexpr uint16_t SPEED_REGISTER_ADDRESS = 0x0032;           // 8.1.3
    static constexpr uint16_t MOTOR_DIRECTION_REGISTER_ADDRESS = 0x0086; // 8.2.10
    static constexpr uint16_t MOTOR_ZERO_AXIS_REGISTER_ADDRESS = 0x0092; // 8.2.19
    static constexpr uint16_t ENABLE_REGISTER_ADDRESS = 0x00F3;          // 8.2.19
    static constexpr uint16_t HOME_REGISTER_ADDRESS_START = 0x0090;      // 8.2.20
    static constexpr uint16_t HOME_Trigger_REGISTER_ADDRESS = 0x0091;    // 8.3.2

    static constexpr uint16_t RELATIVE_POSITION_REGISTER_ADDRESS = 0x00FD; // 8.3.4
    static constexpr uint16_t ABSOLUTE_POSITION_REGISTER_ADDRESS = 0x00FE; // 8.3.5
    // static constexpr uint16_t ENCODER_REGISTER_ADDRESS_START = 0x602C;      // Pr8.44-8.45
    // static constexpr uint16_t PATH_REGISTER_ADDRESS_START = 0x6200;         // Pr9.00-9.07

    static constexpr uint16_t ACCELERATION = 0x240;
    static constexpr double ENCODER_TO_PULSES = 16384.0 / 3200.0;

    int move(int32_t pos, uint16_t vel, bool relative = false);

    int home();

    /* Configuration Options */
    bool isForwardLimit = false;
    uint16_t homeVelocity = 0x50; // 50rpm
    // Bit2: switch type (=0:limit =1:home); Bit1: move after? (=0:No =1:Yes); Bit0: Dir (=0:CCW =1:CW);
    uint16_t homeConfig = 0b010; // limit, yes, CCW

    /* State */
    ModbusDevice modbus;
    bool isHoming = false;
    double lastSecond = 0.0;
};

} // namespace arm_motor_controller

#endif // SERVO42D_HPP