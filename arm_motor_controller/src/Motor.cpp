#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "arm_motor_controller/Motor.hpp"

namespace arm_motor_controller {

Motor::Motor(uint8_t id, double startingPos) : id(id), currentPos(startingPos) {
    RCLCPP_DEBUG(rclcpp::get_logger("MotorState"), "Motor %d created", id);
};

int Motor::enable() { return 0; }
int Motor::disable(bool isEmergency) { return 0; }

int Motor::exportState(std::vector<hardware_interface::StateInterface> &stateInterfaces, std::string jointName) {
    stateInterfaces.emplace_back(jointName, hardware_interface::HW_IF_POSITION, &currentPos);
    stateInterfaces.emplace_back(jointName, hardware_interface::HW_IF_VELOCITY, &currentVel);
    return 0;
}

int Motor::exportCommand(std::vector<hardware_interface::CommandInterface> &commandInterfaces, std::string jointName) {
    commandInterfaces.emplace_back(jointName, hardware_interface::HW_IF_POSITION, &targetPos);
    commandInterfaces.emplace_back(jointName, hardware_interface::HW_IF_VELOCITY, &targetVel);
    return 0;
}

int Motor::read(double time, double period) { return 0; };

int Motor::write(double time, double period) {
    if (targetPos != currentPos) {
        RCLCPP_DEBUG(rclcpp::get_logger("MotorState"), "Position updated for joint %d, was %.4f, now %.4f", id,
                     currentPos, targetPos);
    }
    if (targetVel != currentVel) {
        RCLCPP_DEBUG(rclcpp::get_logger("MotorState"), "Velocity updated for joint %d, was %.4f, now %.4f", id,
                     currentVel, targetVel);
    }
    currentPos = targetPos;
    currentVel = targetVel;

    return 0;
};

} // namespace arm_motor_controller