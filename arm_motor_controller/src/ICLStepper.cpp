#include <math.h>

#include "rclcpp/rclcpp.hpp"

#include "arm_motor_controller/ICLStepper.hpp"

namespace arm_motor_controller {

ICLStepper::ICLStepper(std::shared_ptr<RS485> rs485, uint32_t id, bool isReversed)
    : Motor(rs485, id, isReversed), modbus(id, *rs485, ERROR_DECAY_RATE) {
    motorVel = 0;
    motorPos = 0;
}

ICLStepper::~ICLStepper() {}

void ICLStepper::setMotorHome(uint32_t speed, uint32_t config) {
    homeVelocity = speed;
    homeConfig = config & 0xF;
    isForwardLimit = config & 0x10;
}

int ICLStepper::configure() {
    constexpr int CONFIG_LEN = 12;
    uint16_t config[CONFIG_LEN][2] = {
        {MOTOR_DIRECTION_REGISTER_ADDRESS, (uint16_t)(isReversed ? 1 : 0)},              // Positive direction(0 = CW, 1 = CCW)
        {DIGITAL_IN1_CONFIG_REGISTER_ADDRESS, (uint16_t)(isForwardLimit ? 0x25 : 0x26)}, // DI1 mode (0x26 = neg sw, 0x25 = pos)
        {DIGITAL_IN7_CONFIG_REGISTER_ADDRESS, 0x08},                                     // N.O Enable pin (force disable default)
        {ALARM_CONFIG_REGISTER_ADDRESS, 0x00},                                           // Shutup Alarm
        {HOME_REGISTER_ADDRESS_START + 0, homeConfig},                                   // Home mode
        {HOME_REGISTER_ADDRESS_START + 1, 0x00},                                         // Home Switch Pos High?
        {HOME_REGISTER_ADDRESS_START + 2, 0x00},                                         // Home Switch Pos Low?
        {HOME_REGISTER_ADDRESS_START + 3, 0x00},                                         // Home Move Pos High
        {HOME_REGISTER_ADDRESS_START + 4, 0x00},                                         // Home Move Pos Low
        // skip 0x600F
        {HOME_REGISTER_ADDRESS_START + 6, homeVelocity}, // Home velocity
        {HOME_REGISTER_ADDRESS_START + 7, ACCELERATION}, // Home acc
        {HOME_REGISTER_ADDRESS_START + 8, ACCELERATION}, // Home dec
    };

    rclcpp::Rate rate(10);

    // return 0;
    // update all the config parameters of the motor if there is a deference and save them to the motor.
    bool errorEncountered = false;
    bool saveReg = false;
    uint16_t resp;
    uint16_t val;
    uint16_t reg;
    for (int i = 0; i < CONFIG_LEN; i++) {
        reg = config[i][0];
        val = config[i][1];

        if (modbus.readRegisters(reg, READ_FUNCTION_CODE, 1, &resp)) {
            RCLCPP_WARN(rclcpp::get_logger("MotorState"), "Motor %d unable to read register 0x%X during config, overwriting...",
                        id, reg);
            resp = 0xFFFF;
            errorEncountered = true;
        }

        rate.sleep();

        if (resp != val) {
            RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Motor %d register difference at 0x%X setting to 0x%X from 0x%X", id,
                        reg, val, resp);
            saveReg = true;
            for(int r = 0; r < 2; r++ ){
                if (!modbus.writeRegister(reg, WRITE_FUNCTION_CODE, val)) {
                    break;
                }

                if(r > 0){
                    RCLCPP_ERROR(rclcpp::get_logger("MotorState"), "Motor %d unable to set register 0x%X to 0x%X", id, reg, val);
                    errorEncountered = true; 
                }
            }
        }
    }

    rate.sleep();

    if (saveReg && !errorEncountered) {
        RCLCPP_WARN(rclcpp::get_logger("MotorState"), "Motor %d config differed from desired, saving...", id);

        // TODO save params!
        if (modbus.writeRegister(CONTROL_WORD_REGISTER_ADDRESS, WRITE_FUNCTION_CODE, 0x2211)) {
            RCLCPP_ERROR(rclcpp::get_logger("MotorState"), "Motor %d unable to save registers!!!", id);
            errorEncountered = true;
        }
    }

    return errorEncountered ? -1 : 0;
}

int ICLStepper::enable() {
    Motor::enable();

    // move(0, 0, true); // Stop any movement befor enabling
    modbus.writeRegister(ENABLE_REGISTER_ADDRESS_START, WRITE_FUNCTION_CODE, 0x01);
    move(0,0);
    return 0;
}

int ICLStepper::disable(bool isEmergency) {
    modbus.writeRegister(ENABLE_REGISTER_ADDRESS_START, WRITE_FUNCTION_CODE, 0x00);
    return 0;
}

int ICLStepper::home() {
    if (isHoming) { // Currently homing, check status
        uint16_t resp;
        if (modbus.readRegisters(STATUS_REGISTER_ADDRESS, READ_FUNCTION_CODE, 1, &resp)) {
            RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Motor %d unable to read home status", id);
            isHoming = false;
            return 0;
        }

        if (resp & 0x40) { // Finished homing
            isHoming = false;
            return 1;
        }

    } else { // Start homing
        RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Motor %d is homing...", id);
        move(0, 0, true);
        int ret = modbus.writeRegister(PATH_CONTROL_REGISTER_ADDRESS, WRITE_FUNCTION_CODE, 0x20);
        if (ret != 0) {
            RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Motor %d HOME REQUEST FAILED", id);
            return ret;
        }
        isHoming = true;
    }
    
    return 0;
}

int ICLStepper::move(int32_t pos, uint16_t vel, bool relative) {
    // Build the Immediate Trigger message
    uint16_t movementMessage[8] = {0};
    movementMessage[0] = PATH_MODE | (relative ? 0x40 : 0);
    movementMessage[1] = pos >> 16;
    movementMessage[2] = pos;
    movementMessage[3] = vel;
    movementMessage[4] = ACCELERATION;
    movementMessage[5] = ACCELERATION;
    movementMessage[6] = PATH_DELAY;
    movementMessage[7] = PATH_TRIGGER;

    int ret = modbus.writeMultiple(PATH_REGISTER_ADDRESS_START, WRITE_MULTIPLE_FUNCTION_CODE, 8, movementMessage);
    if (ret) {
        RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Motor %d failed to move to position", id);
    }

    return ret;
}

int ICLStepper::read(double time, double period) {
    // RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Motor %d read update", id);
    // static int count = 0;

    errorCounts = modbus.update(period);

    if ((time - lastSecond) > 1.0) {
        lastSecond = time;
        if (errorCounts > 50) {
            RCLCPP_ERROR(rclcpp::get_logger("MotorState"), "Unable to communicate with Motor %d! counts at %d", id, errorCounts);
        }
    }
    // if (count < 10) {
    //     count++;
    //     return 0;
    // }

    // count = 0;

    uint16_t raw[2] = {0};
    // int ret = readRegister(ENCODER_REGISTER_ADDRESS_START, 2, raw);
    if (modbus.readRegisters(ENCODER_REGISTER_ADDRESS_START, READ_FUNCTION_CODE, 2, raw)) {
        RCLCPP_WARN(rclcpp::get_logger("MotorState"), "Motor %d encountered an error while reading", id);
        return -1;
    }

    double pos = static_cast<double>(raw[0] << 16 | raw[1]);

    rosCurrentPos = motorPos2Radians(pos);
    rosCurrentVel = motorVel2Radians(motorVel);

    // if (count > 30) {
    //     RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Motor %d pos: %0.4f", id, pos);
    //     count = 0;
    // }

    // count++;

    return 0;
}

int ICLStepper::write(double time, double period) {
    (void)time, (void)period;

    // Trigger homing sequence!
    if (rosTriggerHome >= 1.0) {
        int ret = home();
        if(ret == 0){
            rosTriggerHome = 0.0;
            rosIsHomed = 0.0;
        }
        return ret;
    }

    if (isHoming) { // Handle homing
        int ret;
        if ((ret = home()) > 0) { // Finished homing
            rosIsHomed = 1.0;
        } else if (ret < 0) { // Error homing
            RCLCPP_ERROR(rclcpp::get_logger("MotorState"), "Motor %d homing failed! %d", id, ret);
            return ret;
        }

    } else { // Normal Move
        double pos = radians2MotorPos(rosTargetPos);
        double vel = radians2MotorVel(rosTargetVel);

        // Only update the pos on significant change
        if (abs(pos - motorPos) < 1) {
            return 0;
        } else if (abs(vel) < 0.00001) { // Handle stops
            motorVel = 0.0; 
            return move(0,0);
        } else if (signbit(pos - motorPos) != signbit(vel)) {
            // Only move to position in the same direction as the velocity (Fixes weirdness from JTC)
            return 0;
        }

        // RCLCPP_INFO(rclcpp::get_logger("MotorState"),
        //             "Motor %d update to position %0.4f (%0.4f), error [%0.4f] with velocity %0.4f (%0.4f)", id, pos, rosTargetPos,
        //             rosTargetPos - rosCurrentPos, vel, rosTargetVel);

        uint16_t realVel = abs(vel) < 1.0 ? 1 : abs(vel);

        if (move(pos, realVel)) {
            return -1;
        }

        motorVel = realVel;
        motorPos = trunc(pos);

        // std::stringstream ss;
        // ss << std::hex << std::setfill('0');
        // for (size_t i = 0; i < ARRAY_LEN(movementMessage); ++i) {
        //     ss << std::setw(2) << static_cast<int>(movementMessage[i]);
        // }
        // std::string str = ss.str();

        // RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Motor %d moving to position %d with velocity %d for message
        // %s", id, realPos, realVel, str.c_str());
    }
    return 0;
}

} // namespace arm_motor_controller