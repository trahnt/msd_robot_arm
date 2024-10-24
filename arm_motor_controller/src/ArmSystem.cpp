#include <string>

#include "rclcpp/rclcpp.hpp"

#include "arm_motor_controller/ArmSystem.hpp"
#include "arm_motor_controller/Motor.hpp"

namespace arm_motor_controller {

hardware_interface::CallbackReturn ArmSystemHardware::on_init(const hardware_interface::HardwareInfo &info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    // hw_start_sec_ = hardware_interface::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);

    for (const auto &joint : info.joints) {

        // Do som config error checking before initializing a motor
        // Command interface checking
        if (joint.command_interfaces.size() != 2) {
            RCLCPP_FATAL(rclcpp::get_logger("ArmController"),
                         "Joint '%s' has %zu command interfaces found. 2 expected.", joint.name.c_str(),
                         joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
            joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(rclcpp::get_logger("ArmController"),
                         "Joint '%s' must have position and velocity command interface", joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // State interface checking
        if (joint.state_interfaces.size() != 2) {
            RCLCPP_FATAL(rclcpp::get_logger("ArmController"), "Joint '%s' has %zu state interfaces found. 2 expected.",
                         joint.name.c_str(), joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
            joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(rclcpp::get_logger("ArmController"),
                         "Joint '%s' must have position and velocity state interface", joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        std::string motorType;
        uint8_t deviceID;
        double startingPos = 0;

        if (auto search = joint.parameters.find("type"); search != joint.parameters.end()) {
            motorType = search->second;
        } else {
            RCLCPP_FATAL(rclcpp::get_logger("ArmController"), "Joint '%s' missing parameter \"type\"",
                         joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (auto search = joint.parameters.find("id"); search != joint.parameters.end()) {
            deviceID = std::stoi(search->second);
        } else {
            RCLCPP_FATAL(rclcpp::get_logger("ArmController"), "Joint '%s' missing parameter \"id\"",
                         joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // if (auto search = joint.parameters.find("initial_value"); search != joint.parameters.end()) {
        //     startingPos = std::stof(search->second);
        // } else {
        //     RCLCPP_FATAL(rclcpp::get_logger("ArmController"), "Joint '%s' missing parameter \"initial_va2\"",
        //                  joint.name.c_str());
        //     return hardware_interface::CallbackReturn::ERROR;
        // }

        motors.emplace(joint.name, Motor(deviceID, startingPos));
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArmSystemHardware::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (auto motor : motors) {
        motor.second.exportState(state_interfaces, motor.first);
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArmSystemHardware::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (auto motor : motors) {
        motor.second.exportCommand(command_interfaces, motor.first);
    }

    return command_interfaces;
}

hardware_interface::CallbackReturn ArmSystemHardware::on_configure(const rclcpp_lifecycle::State &previous_state) {
    (void)previous_state;
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmSystemHardware::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
    (void)previous_state;
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmSystemHardware::on_activate(const rclcpp_lifecycle::State &previous_state) {
    (void)previous_state;
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmSystemHardware::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    (void)previous_state;
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmSystemHardware::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
    (void)previous_state;
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArmSystemHardware::read(const rclcpp::Time &time, const rclcpp::Duration &period) {
    for (auto motor : motors) {
        motor.second.read(time.seconds(), period.seconds());
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArmSystemHardware::write(const rclcpp::Time &time, const rclcpp::Duration &period) {
    for (auto motor : motors) {
        motor.second.write(time.seconds(), period.seconds());
    }

    return hardware_interface::return_type::OK;
}

} // namespace arm_motor_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(arm_motor_controller::ArmSystemHardware, hardware_interface::SystemInterface)