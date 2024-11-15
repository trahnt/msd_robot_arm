#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "arm_motor_controller/Motor.hpp"

namespace arm_motor_controller {

Motor::Motor(uint8_t id, double startingPos) : id(id), currentPos(startingPos) {
    RCLCPP_DEBUG(rclcpp::get_logger("MotorState"), "Motor %d created", id);
    targetPos = startingPos;
};

int Motor::enable() {
    RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Enabling joint %d at position, current %.4f, target %.4f", id,
                currentPos, targetPos);
    return 0;
}
int Motor::disable(bool isEmergency) {
    RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Disabling joint %d at position, current %.4f, target %.4f", id,
                currentPos, targetPos);
    return 0;
}

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

int Motor::read(double time, double period) { 
    // if(time - lastupdate > 1.0){
    //     RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Current position for joint %d is %e.", id, currentPos);
    //     lastupdate = time;
    // }

    return 0; 
};

int Motor::write(double time, double period) {
    // if (abs(targetPos - currentPos) > 0.0001) {
    //     RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Position updated for joint %d, was %.4f, now %.4f.", id,
    //                 currentPos, targetPos);
    // }
    // if (abs(targetVel - currentVel) > 0.0001) {
    //     RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Velocity updated for joint %d, was %.4f, now %.4f.", id,
    //                 currentVel, targetVel);
    // }
    currentPos = targetPos;
    currentVel = targetVel;

    return 0;
};

} // namespace arm_motor_controller