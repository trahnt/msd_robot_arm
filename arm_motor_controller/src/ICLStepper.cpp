#include "rclcpp/rclcpp.hpp"

#include "arm_motor_controller/ICLStepper.hpp"

namespace arm_motor_controller {

ICLStepper::ICLStepper(std::shared_ptr<RS485> rs485, uint32_t id, double startingPos)
    : Motor(rs485, id, startingPos), modbus(id, *rs485, ERROR_DECAY_RATE) {
    motorVel = 0;
    motorPos = startingPos;
}

ICLStepper::~ICLStepper() {}

void ICLStepper::setMotorHome(uint32_t speed, uint32_t config) {
    homeVelocity = speed;
    homeConfig = config;
}

int ICLStepper::configure() {
    constexpr uint16_t config[2][2] = {
        {0x0145, 0x26}, // DI1 mode (0x26 = neg sw, 0x25 = pos)
        {0x016D, 0x0},  // Alarm (0 = shut up)
    };

    modbus.writeRegister(config[0][0], WRITE_FUNCTION_CODE, config[0][1]);

    // uint16_t test[] = {0x0000, 0x0028, 0x0000, 0x0029};
    // RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Motor %d multi-write: %d", id, writeMultiple(0x0146, 4, test));

    return 0;
}

int ICLStepper::enable() {
    Motor::enable();

    move(0, 0); // Stop any movement befor enabling
    modbus.writeRegister(ENABLE_REGISTER_ADDRESS_START, WRITE_FUNCTION_CODE, 0x01);

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
            return 0;
        }

        if (resp & 0x40) { // Finished homing
            return 1;
        }

    } else { // Start homing
        int ret;
        if ((ret = modbus.writeRegister(PATH_CONTROL_REGISTER_ADDRESS, WRITE_FUNCTION_CODE, 0x20)) != 0) {
            return ret;
        }
        isHoming = true;
    }
    return 0;
}

int ICLStepper::move(int32_t pos, uint16_t vel) {
    // Build the Immediate Trigger message
    uint16_t movementMessage[8] = {0};
    movementMessage[0] = PATH_MODE;
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
            RCLCPP_ERROR(rclcpp::get_logger("MotorState"), "Unable to communicate with Motor %d! counts at %d", id,
                         errorCounts);
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
        // RCLCPP_WARN(rclcpp::get_logger("MotorState"), "Motor %d encountered and error while reading, got %d", id,
        // ret);
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
        rosTriggerHome = 0.0;
        rosIsHomed = 0.0;
        return home();
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
        }
        RCLCPP_INFO(rclcpp::get_logger("MotorState"),
                    "Motor %d update to position %0.4f (%0.4f) with velocity %0.4f (%0.4f)", id, pos, rosTargetPos, vel,
                    rosTargetVel);

        uint16_t realVel = abs(vel) < 1.0 ? 1 : abs(vel);

        if (!move(pos, realVel)) {
            motorVel = realVel;
            motorPos = trunc(pos);
        }

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