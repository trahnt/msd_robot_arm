#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "arm_motor_controller/Motor.hpp"

#include <chrono>
#include <thread>

namespace arm_motor_controller {

Motor::Motor(std::shared_ptr<RS485> rs485, uint8_t id, bool isReversed) : rs485(rs485), id(id), isReversed(isReversed) {
    RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Motor %d created", id);
    rosTargetPos = 0.0;
    rosIsHomed = 0.0;
};

int Motor::enable() {
    RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Enabling motor %d at position, current %.4f, target %.4f", id,
                rosCurrentPos, rosTargetPos);
    return 0;
}

int Motor::disable(bool isEmergency) {
    (void)isEmergency;

    RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Disabling motor %d at position, current %.4f, target %.4f", id,
                rosCurrentPos, rosTargetPos);
    return 0;
}

int Motor::exportState(std::vector<hardware_interface::StateInterface> &stateInterfaces, std::string jointName) {
    stateInterfaces.emplace_back(jointName, hardware_interface::HW_IF_POSITION, &rosCurrentPos);
    stateInterfaces.emplace_back(jointName, hardware_interface::HW_IF_VELOCITY, &rosCurrentVel);
    stateInterfaces.emplace_back(jointName, arm_motor_controller::HW_IF_HOME, &rosIsHomed);
    return 0;
}

int Motor::exportCommand(std::vector<hardware_interface::CommandInterface> &commandInterfaces, std::string jointName) {
    commandInterfaces.emplace_back(jointName, hardware_interface::HW_IF_POSITION, &rosTargetPos);
    commandInterfaces.emplace_back(jointName, hardware_interface::HW_IF_VELOCITY, &rosTargetVel);
    commandInterfaces.emplace_back(jointName, arm_motor_controller::HW_IF_HOME, &rosTriggerHome);

    RCLCPP_INFO(rclcpp::get_logger("MotorState"),
                "Motor %d command exported with joint range (%.4f,%.4f) and motor range(%.4f,%.4f)", id, jointLimits[0],
                jointLimits[1], motorLimits[0], motorLimits[1]);
    return 0;
}

int Motor::read(double time, double period) {
    (void)time, (void)period;
    // RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Motor %d read update", id);
    rosCurrentPos = motorPos2Radians(motorPos);
    rosCurrentVel = motorVel2Radians(motorVel);

    if (rosTriggerHome >= 1.0) {
        home();
    }

    return 0;
};

int Motor::write(double time, double period) {
    (void)time, (void)period;
    motorPos = radians2MotorPos(rosTargetPos);
    motorVel = radians2MotorVel(rosTargetVel);

    // if (abs(rosTargetPos - rosCurrentPos) > 0.0001) {
    //     RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Position updated for joint %d, was %.4f, now %.4f (%.4f).",
    //     id,
    //                 rosCurrentPos, rosTargetPos, motorPos);
    // }
    // if (abs(rosTargetVel - rosCurrentVel) > 0.0001) {
    //     RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Velocity updated for joint %d, was %.4f, now %.4f (%.4f).",
    //     id,
    //                 rosCurrentVel, rosTargetVel, motorVel);
    // }

    return 0;
};

int Motor::home(){
    RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Motor %d is homing...", id);
    rosIsHomed = 0.0;
    rosTriggerHome = 0.0;
    // maybe add a time.sleep?

    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    RCLCPP_INFO(rclcpp::get_logger("MotorState"), "...Motor %d home!", id);
    rosIsHomed = 1.0;
    return 0;
}

} // namespace arm_motor_controller
