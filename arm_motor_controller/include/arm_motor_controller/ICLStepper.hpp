#ifndef ICL_STEPPER_HPP
#define ICL_STEPPER_HPP

#include "arm_motor_controller/Communication/ModbusDevice.hpp"
#include "arm_motor_controller/Motor.hpp"

namespace arm_motor_controller {

class ICLStepper : public Motor {

public:
    ICLStepper(std::shared_ptr<RS485> rs485, uint32_t id, bool isReversed = false);
    ~ICLStepper();

    void setMotorHome(uint32_t speed, uint32_t config) override;

    int configure() override;

    int enable() override;
    int disable(bool isEmergency = false) override;

    int read(double time, double period) override;
    int write(double time, double period) override;

private:
    /* Modbus constants */
    static constexpr double ERROR_DECAY_RATE = 0.25;
    static constexpr uint8_t READ_FUNCTION_CODE = 0x03;
    static constexpr uint8_t WRITE_FUNCTION_CODE = 0x06;
    static constexpr uint8_t WRITE_MULTIPLE_FUNCTION_CODE = 0x10;

    /* Configuration Registers */
    static constexpr uint16_t MOTOR_DIRECTION_REGISTER_ADDRESS = 0x0007;    // Pr0.03
    static constexpr uint16_t DIGITAL_IN1_CONFIG_REGISTER_ADDRESS = 0x0145; // Pr4.02
    static constexpr uint16_t DIGITAL_IN7_CONFIG_REGISTER_ADDRESS = 0x0151; // Pr4.08
    static constexpr uint16_t ALARM_CONFIG_REGISTER_ADDRESS = 0x016D;       // Pr4.22
    static constexpr uint16_t STATUS_REGISTER_ADDRESS = 0x1003;             // Pr---
    static constexpr uint16_t CONTROL_WORD_REGISTER_ADDRESS = 0x1801;       // Pr---
    static constexpr uint16_t PATH_CONTROL_REGISTER_ADDRESS = 0x6002;       // Pr8.02
    static constexpr uint16_t HOME_REGISTER_ADDRESS_START = 0x600A;         // Pr8.10-8.18
    static constexpr uint16_t ENCODER_REGISTER_ADDRESS_START = 0x602C;      // Pr8.44-8.45
    static constexpr uint16_t PATH_REGISTER_ADDRESS_START = 0x6200;         // Pr9.00-9.07
    static constexpr uint16_t ENABLE_REGISTER_ADDRESS_START = 0x000F;       // Pr0.07

    /* Path Movement options */
    static constexpr uint16_t PATH_MODE = 0x01; // Abs move, Overlaying, Interrupt, Position move
    static constexpr uint16_t PATH_DELAY = 0;
    static constexpr uint16_t PATH_TRIGGER = 0x10;

    static constexpr uint8_t ACCELERATION = 0x5;
    static constexpr uint32_t PULSES_PER_REV = 10000;

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

#endif // ICL_STEPPER_HPP