#ifndef ARM_SYSTEM_HPP
#define ARM_SYSTEM_HPP

#include <map>
#include <memory>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "arm_motor_controller/Communication/RS485.hpp"
#include "arm_motor_controller/Motor.hpp"

namespace arm_motor_controller {

class ArmSystemHardware : public hardware_interface::SystemInterface {
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(ArmSystemHardware)

    static constexpr char ICL_MOTOR[] = "iCL";
    static constexpr char SERVO42D_MOTOR[] = "Servo42D";
    static constexpr char SERVO57C_MOTOR[] = "Servo57C";

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
    // Parameters for the DiffBot simulation
    double hw_start_sec_;
    double hw_stop_sec_;

    // Store the command for the simulated robot
    std::map<std::string, std::unique_ptr<Motor>> motors;

    // Shared RS485 connection
    std::shared_ptr<RS485> rs485;
};

} // namespace arm_motor_controller

#endif // ARM_SYSTEM_HPP