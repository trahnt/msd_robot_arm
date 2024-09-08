
#include "arm_motor_controller/ArmSystem.hpp"

namespace arm_motor_controller {

hardware_interface::CallbackReturn ArmSystemHardware::on_init(const hardware_interface::HardwareInfo &info) {
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArmSystemHardware::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArmSystemHardware::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    return command_interfaces;
}

hardware_interface::CallbackReturn ArmSystemHardware::on_configure(const rclcpp_lifecycle::State &previous_state) {
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmSystemHardware::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmSystemHardware::on_activate(const rclcpp_lifecycle::State &previous_state) {
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmSystemHardware::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmSystemHardware::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArmSystemHardware::read(const rclcpp::Time &time, const rclcpp::Duration &period) {
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArmSystemHardware::write(const rclcpp::Time &time, const rclcpp::Duration &period) {
    return hardware_interface::return_type::OK;
}

} // namespace arm_motor_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(arm_motor_controller::ArmSystemHardware, hardware_interface::SystemInterface)