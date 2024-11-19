#include <any>
#include <map>
#include <string>

#include "joint_limits/joint_limits.hpp"
#include "joint_limits/joint_limits_rosparam.hpp"
#include "rclcpp/rclcpp.hpp"
#include "urdf/model.h"

#include "arm_motor_controller/ArmSystem.hpp"
#include "arm_motor_controller/Motor.hpp"
#include "arm_motor_controller/Servo57C.hpp"

namespace arm_motor_controller {

enum class ParameterType { Int, Double, String };

hardware_interface::CallbackReturn ArmSystemHardware::on_init(const hardware_interface::HardwareInfo &info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    urdf::Model model;
    if (!model.initString(info.original_xml)) {
        RCLCPP_FATAL(rclcpp::get_logger("ArmController"), "Failed to load urdf!");
        return hardware_interface::CallbackReturn::ERROR;
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
        int deviceID;
        double motorLimits[2];
        double RPM;

        double startingPos = 0;

        std::map<std::string, std::pair<ParameterType, std::any>> parameters = {
            {"type", std::make_pair(ParameterType::String, &motorType)},
            {"id", std::make_pair(ParameterType::Int, &deviceID)},
            {"min", std::make_pair(ParameterType::Double, &motorLimits[0])},
            {"max", std::make_pair(ParameterType::Double, &motorLimits[1])},
            {"rpm", std::make_pair(ParameterType::Double, &RPM)},
        };

        for (const auto &param : parameters) {
            auto search = joint.parameters.find(param.first);
            if (search == joint.parameters.end()) {
                RCLCPP_FATAL(rclcpp::get_logger("ArmController"), "Joint '%s' missing parameter \"%s\"",
                             joint.name.c_str(), param.first.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }

            switch (param.second.first) {
            case ParameterType::Int:
                *std::any_cast<int *>(param.second.second) = std::stoi(search->second);
                break;
            case ParameterType::Double:
                *std::any_cast<double *>(param.second.second) = std::stof(search->second);
                break;
            case ParameterType::String:
                *std::any_cast<std::string *>(param.second.second) = search->second;
                break;
            default:
                break;
            }
        }

        urdf::JointLimitsSharedPtr jointLimits = model.getJoint(joint.name.c_str())->limits;

        // Create the corrsponding motor type for the joint
        if(motorType.compare("iCL") == 0){
            motors.insert(
                std::make_pair(joint.name, std::make_unique<Motor>(deviceID, jointLimits->lower, jointLimits->upper,
                                                                   motorLimits[0], motorLimits[1], startingPos)));
        } else if (motorType.compare("Servo57C") == 0) {
            // motors.insert(
            //     std::make_pair(joint.name, std::make_unique<Motor>(deviceID, jointLimits->lower, jointLimits->upper,
                                                                //    motorLimits[0], motorLimits[1], startingPos)));
            motors.insert(
                std::make_pair(joint.name, std::make_unique<Servo57C>(deviceID, jointLimits->lower, jointLimits->upper,
                                                                      motorLimits[0], motorLimits[1], startingPos)));

        } else if (motorType.compare("none") == 0) {
            motors.insert(
                std::make_pair(joint.name, std::make_unique<Motor>(deviceID, jointLimits->lower, jointLimits->upper,
                                                                   motorLimits[0], motorLimits[1], startingPos)));
        } else { // Invalid type
            RCLCPP_FATAL(rclcpp::get_logger("ArmController"), "Joint '%s' has invalid type \"%s\"", joint.name.c_str(),
                         motorType.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("ArmController"), "Joint '%s' created with '%s' motor", joint.name.c_str(),
                    motorType.c_str());
    }

    RCLCPP_INFO(rclcpp::get_logger("ArmController"), "Initalized hardware with %ld motors", motors.size());

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArmSystemHardware::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (auto const &motor : motors) {
        motor.second->exportState(state_interfaces, motor.first);
        RCLCPP_INFO(rclcpp::get_logger("ArmController"), "Joint '%s' state exported", motor.first.c_str());
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArmSystemHardware::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (auto const &motor : motors) {
        motor.second->exportCommand(command_interfaces, motor.first);
        RCLCPP_INFO(rclcpp::get_logger("ArmController"), "Joint '%s' command exported", motor.first.c_str());
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

    for (auto const &motor : motors) {
        motor.second->enable();
        RCLCPP_INFO(rclcpp::get_logger("ArmController"), "Joint '%s' enabled", motor.first.c_str());
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmSystemHardware::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    (void)previous_state;

    for (auto const &motor : motors) {
        motor.second->disable();
        RCLCPP_INFO(rclcpp::get_logger("ArmController"), "Joint '%s' disabled", motor.first.c_str());
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmSystemHardware::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
    (void)previous_state;
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArmSystemHardware::read(const rclcpp::Time &time, const rclcpp::Duration &period) {
    static double lastupdate = 0.0;
    if (time.seconds() - lastupdate > 1) {
        RCLCPP_INFO(rclcpp::get_logger("ArmController"), "Got read update! Current time: %f, Period: %f",
                    time.seconds(), period.seconds());
        lastupdate = time.seconds();
    }

    for (auto const &motor : motors) {
        motor.second->read(time.seconds(), period.seconds());
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArmSystemHardware::write(const rclcpp::Time &time, const rclcpp::Duration &period) {
    static double lastupdate = 0.0;
    if (time.seconds() - lastupdate > 1) {
        RCLCPP_INFO(rclcpp::get_logger("ArmController"), "Got write update! Current time: %f, Period: %f",
                    time.seconds(), period.seconds());
        lastupdate = time.seconds();
    }

    for (auto const &motor : motors) {
        motor.second->write(time.seconds(), period.seconds());
    }

    return hardware_interface::return_type::OK;
}

} // namespace arm_motor_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(arm_motor_controller::ArmSystemHardware, hardware_interface::SystemInterface)