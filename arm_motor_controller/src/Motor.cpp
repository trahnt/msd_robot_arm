#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "arm_motor_controller/Motor.hpp"

namespace arm_motor_controller {

Motor::Motor(uint32_t id, double jointMin, double jointMax, double motorMin, double motorMax, double startingPos)
    : id(id), jointLimits{jointMin, jointMax}, motorLimits{motorMin, motorMax}, rosCurrentPos(startingPos) {
    RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Motor %d created for joint range (%.4f,%.4f) to motor range(%.4f,%.4f)",
                id, jointLimits[0], jointLimits[1], motorLimits[0], motorLimits[1]);
    rosTargetPos = startingPos;
};

int Motor::enable() {
    RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Enabling joint %d at position, current %.4f, target %.4f", id,
                rosCurrentPos, rosTargetPos);
    return 0;
}
int Motor::disable(bool isEmergency) {
    RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Disabling joint %d at position, current %.4f, target %.4f", id,
                rosCurrentPos, rosTargetPos);
    return 0;
}

int Motor::exportState(std::vector<hardware_interface::StateInterface> &stateInterfaces, std::string jointName) {
    stateInterfaces.emplace_back(jointName, hardware_interface::HW_IF_POSITION, &rosCurrentPos);
    stateInterfaces.emplace_back(jointName, hardware_interface::HW_IF_VELOCITY, &rosCurrentVel);
    return 0;
}

int Motor::exportCommand(std::vector<hardware_interface::CommandInterface> &commandInterfaces, std::string jointName) {
    commandInterfaces.emplace_back(jointName, hardware_interface::HW_IF_POSITION, &rosTargetPos);
    commandInterfaces.emplace_back(jointName, hardware_interface::HW_IF_VELOCITY, &rosTargetVel);
    return 0;
}

int Motor::read(double time, double period) {

    rosCurrentPos = motorPos2Radians(motorPos);
    rosCurrentVel = motorVel2Radians(motorVel);

    // if(time - lastupdate > 1.0){
    //     RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Current position for joint %d is %e.", id, currentPos);
    //     lastupdate = time;
    // }

    return 0;
};

int Motor::write(double time, double period) {
    motorPos = radians2MotorPos(rosTargetPos);
    motorVel = radians2MotorVel(rosTargetVel);

    if (abs(rosTargetPos - rosCurrentPos) > 0.0001) {
        RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Position updated for joint %d, was %.4f, now %.4f (%.4f).", id,
                    rosCurrentPos, rosTargetPos, motorPos);
    }
    if (abs(rosTargetVel - rosCurrentVel) > 0.0001) {
        RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Velocity updated for joint %d, was %.4f, now %.4f (%.4f).", id,
                    rosCurrentVel, rosTargetVel, motorVel);
    }
    

    return 0;
};

} // namespace arm_motor_controller