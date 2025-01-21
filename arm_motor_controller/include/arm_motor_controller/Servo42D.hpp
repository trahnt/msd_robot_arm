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
    static constexpr uint16_t ENCODER_REGISTER_ADDRESS_START = 0x030;      // 8.1.2
    static constexpr uint16_t SPEED_REGISTER_ADDRESS = 0x0032;             // 8.1.3
    static constexpr uint16_t STATUS_REGISTER_ADDRESS = 0x00F1;            // 8.1.10
    static constexpr uint16_t RESTART_REGISTER_ADDRESS = 0x0041;           // 8.2.3
    static constexpr uint16_t MOTOR_DIRECTION_REGISTER_ADDRESS = 0x0086;   // 8.2.10
    static constexpr uint16_t MOTOR_ZERO_AXIS_REGISTER_ADDRESS = 0x0092;   // 8.2.19
    static constexpr uint16_t ENABLE_REGISTER_ADDRESS = 0x00F3;            // 8.2.19
    static constexpr uint16_t HOME_REGISTER_ADDRESS_START = 0x0090;        // 8.2.20
    static constexpr uint16_t HOME_Trigger_REGISTER_ADDRESS = 0x0091;      // 8.3.2
    static constexpr uint16_t RELATIVE_POSITION_REGISTER_ADDRESS = 0x00FD; // 8.3.4
    static constexpr uint16_t ABSOLUTE_POSITION_REGISTER_ADDRESS = 0x00FE; // 8.3.5

    static constexpr uint16_t ACCELERATION = 240;
    // static constexpr double ENCODER_TO_PULSES = 0x4000 / 1600.0; // 8 Mstep, 3200.0 - 16Mstep
    static constexpr uint16_t ENCODER_VALUE_MAX = 0x4000;
    static constexpr uint32_t PULSES_PER_REV = 1600;
    static constexpr uint16_t FUNNY_VALUE = 0x3a09;
    int move(int32_t pos, uint16_t vel, bool relative = false);

    int home();

    /* Configuration Options */
    bool isHomeHighLimit = false;
    bool homeCCW = false;
    uint16_t homeVelocity = 0x50; // 50rpm
    // Bit2: switch type (=0:limit =1:home); Bit1: move after? (=0:No =1:Yes); Bit0: Dir (=0:CCW =1:CW);
    // uint16_t homeConfig = 0b010; // limit, yes, CCW


    /* State */
    ModbusDevice modbus;
    bool isHoming = false;
    double lastSecond = 0.0;
    double resetOffset = 0.0;
};

} // namespace arm_motor_controller

#endif // SERVO42D_HPP