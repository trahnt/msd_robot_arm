#include <any>
#include <map>
#include <string>
#include <tuple>

#include "joint_limits/joint_limits.hpp"
#include "joint_limits/joint_limits_rosparam.hpp"
#include "rclcpp/rclcpp.hpp"
#include "urdf/model.h"

#include "arm_motor_controller/ArmSystem.hpp"
#include "arm_motor_controller/Communication/RS485.hpp"
#include "arm_motor_controller/ICLStepper.hpp"
#include "arm_motor_controller/Motor.hpp"
#include "arm_motor_controller/Servo42D.hpp"
#include "arm_motor_controller/Servo57C.hpp"

namespace {

enum class ParameterType { Int, Double, String, Bool };

/**
 * Parses a map of parameters to the individual variables and convert it to the correct type
 */
hardware_interface::CallbackReturn
parseParameters(const std::unordered_map<std::string, std::string> parameters,
                const std::map<std::string, std::tuple<ParameterType, std::any, std::any>> parameterMap, std::string logPrefix) {
    for (const auto &param : parameterMap) {
        // Find parameter
        auto search = parameters.find(param.first);
        if (search == parameters.end()) {

            if (std::get<2>(param.second).has_value()) { // No value, use default
                RCLCPP_WARN(rclcpp::get_logger("ArmController"), "%s missing parameter \"%s\", using default", logPrefix.c_str(),
                            param.first.c_str());
                auto val = std::get<2>(param.second);
                switch (std::get<0>(param.second)) {
                case ParameterType::Int:
                    *std::any_cast<int *>(std::get<1>(param.second)) = std::any_cast<int>(val);
                    break;
                case ParameterType::Double:
                    *std::any_cast<double *>(std::get<1>(param.second)) = std::any_cast<double>(val);
                    break;
                case ParameterType::String:
                    *std::any_cast<std::string *>(std::get<1>(param.second)) = std::any_cast<std::string>(val);
                    break;
                case ParameterType::Bool:
                    *std::any_cast<bool *>(std::get<1>(param.second)) = std::any_cast<bool>(val);
                    break;
                default:
                    break;
                }
                continue;
            } else { // No value and no default - > required!
                RCLCPP_FATAL(rclcpp::get_logger("ArmController"), "%s missing parameter \"%s\"", logPrefix.c_str(),
                             param.first.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        // Parse value for parameter
        switch (std::get<0>(param.second)) {
        case ParameterType::Int:
            *std::any_cast<int *>(std::get<1>(param.second)) = std::stoi(search->second);
            break;
        case ParameterType::Double:
            *std::any_cast<double *>(std::get<1>(param.second)) = std::stof(search->second);
            break;
        case ParameterType::String:
            *std::any_cast<std::string *>(std::get<1>(param.second)) = search->second;
            break;
        case ParameterType::Bool:
            *std::any_cast<bool *>(std::get<1>(param.second)) = std::stoi(search->second);
            break;
        default:
            break;
        }
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

} // namespace

namespace arm_motor_controller {

hardware_interface::CallbackReturn ArmSystemHardware::on_init(const hardware_interface::HardwareInfo &info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    urdf::Model model;
    if (!model.initString(info.original_xml)) {
        RCLCPP_FATAL(rclcpp::get_logger("ArmController"), "Failed to load urdf!");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Load RS485 connection settings
    std::string port;
    std::string parityStr;
    int baudrate;
    int byteSize;
    int stopBits;
    int timeout;

    const std::map<std::string, std::tuple<ParameterType, std::any, std::any>> CommunicationParams = {
        {"port", std::make_tuple(ParameterType::String, &port, std::nullopt)},
        {"baudrate", std::make_tuple(ParameterType::Int, &baudrate, std::nullopt)},
        {"parity", std::make_tuple(ParameterType::String, &parityStr, std::nullopt)},
        {"bytesize", std::make_tuple(ParameterType::Int, &byteSize, std::nullopt)},
        {"stopbits", std::make_tuple(ParameterType::Int, &stopBits, std::nullopt)},
        {"timeout", std::make_tuple(ParameterType::Int, &timeout, std::nullopt)},
    };

    if (parseParameters(info.hardware_parameters, CommunicationParams, "RS485 communication") !=
        hardware_interface::CallbackReturn::SUCCESS) {

        return hardware_interface::CallbackReturn::ERROR;
    }

    RS485Parity parity;
    if (parityStr.compare("None") == 0) {
        parity = RS485Parity::None;
    } else if (parityStr.compare("Even") == 0) {
        parity = RS485Parity::Even;
    } else if (parityStr.compare("Odd") == 0) {
        parity = RS485Parity::Odd;
    } else {
        RCLCPP_FATAL(rclcpp::get_logger("ArmController"), "Invalid RS485 parity type: \"%s\"", parityStr.c_str());
    }

    rs485 = std::make_shared<RS485>(port.c_str(), baudrate, parity, byteSize, stopBits, timeout);

    // Load joint configs:
    for (const auto &joint : info.joints) {

        // Do som config error checking before initializing a motor
        // Command interface checking
        if (joint.command_interfaces.size() != 3) {
            RCLCPP_FATAL(rclcpp::get_logger("ArmController"), "Joint '%s' has %zu command interfaces found. 2 expected.",
                         joint.name.c_str(), joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
            joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(rclcpp::get_logger("ArmController"), "Joint '%s' must have position and velocity command interface",
                         joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // State interface checking
        if (joint.state_interfaces.size() != 3) {
            RCLCPP_FATAL(rclcpp::get_logger("ArmController"), "Joint '%s' has %zu state interfaces found. 2 expected.",
                         joint.name.c_str(), joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
            joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(rclcpp::get_logger("ArmController"), "Joint '%s' must have position and velocity state interface",
                         joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        std::string motorType;
        int deviceID;
        double motorLimits[2];
        double RPM;
        double homeSpeed;
        int homeConfig;
        bool isReversed;

        // double startingPos = 0;
        urdf::JointLimitsSharedPtr jointLimits = model.getJoint(joint.name.c_str())->limits;

        // Parse hardware parameters
        const std::map<std::string, std::tuple<ParameterType, std::any, std::any>> jointParams = {
            {"type", std::make_tuple(ParameterType::String, &motorType, std::any())},
            {"id", std::make_tuple(ParameterType::Int, &deviceID, std::any())},
            {"min", std::make_tuple(ParameterType::Double, &motorLimits[0], std::any())},
            {"max", std::make_tuple(ParameterType::Double, &motorLimits[1], std::any())},
            {"rpm", std::make_tuple(ParameterType::Double, &RPM, std::any())},
            {"homeSpeed", std::make_tuple(ParameterType::Double, &homeSpeed, std::make_any<double>(50))},
            {"homeConfig", std::make_tuple(ParameterType::Int, &homeConfig, 0)},
            {"reverse", std::make_tuple(ParameterType::Bool, &isReversed, std::make_any<bool>(false))},
        };

        std::stringstream ss;
        ss << "Joint '" << joint.name << "'";

        if (parseParameters(joint.parameters, jointParams, ss.str()) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        std::unique_ptr<Motor> motor;
        // Create the corrsponding motor type for the joint
        if (motorType.compare(ICL_MOTOR) == 0) {
            motor = std::make_unique<ICLStepper>(rs485, deviceID, isReversed);
        } else if (motorType.compare(SERVO57C_MOTOR) == 0) {
            motor = std::make_unique<Servo57C>(rs485, deviceID, isReversed);
        } else if (motorType.compare(SERVO42D_MOTOR) == 0) {
            motor = std::make_unique<Servo42D>(rs485, deviceID, isReversed);
        } else if (motorType.compare("none") == 0) {
            motor = std::make_unique<Motor>(rs485, deviceID, isReversed);
        } else { // Invalid type
            RCLCPP_FATAL(rclcpp::get_logger("ArmController"), "Joint '%s' has invalid type \"%s\"", joint.name.c_str(),
                         motorType.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        motor->setJointLimits(jointLimits->lower, jointLimits->upper);
        motor->setMotorLimits(motorLimits[0], motorLimits[1]);
        motor->setMotorSpeedScale(RPM);
        motor->setMotorHome(homeSpeed, homeConfig);
        motors.insert(std::make_pair(joint.name, std::move(motor)));

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
    if(!rs485->connect()) {
        RCLCPP_ERROR(rclcpp::get_logger("ArmController"), "Failed to establish serial ocnnection over RS485!");

        // if you want to ignore the problem...
        RCLCPP_WARN(rclcpp::get_logger("ArmController"), "ELECTING TO IGNORE THE FACT SERIAL BROKE");
        return hardware_interface::CallbackReturn::SUCCESS;

        return hardware_interface::CallbackReturn::FAILURE; 
    }

    for (auto const &motor : motors) {
        motor.second->configure();
        RCLCPP_INFO(rclcpp::get_logger("ArmController"), "Joint '%s' configured", motor.first.c_str());
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmSystemHardware::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
    (void)previous_state;
    if (rs485->disconnect()) {
        return hardware_interface::CallbackReturn::SUCCESS;
    } else {
        return hardware_interface::CallbackReturn::FAILURE;
    }
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
        // RCLCPP_INFO(rclcpp::get_logger("ArmController"), "Got read update! Current time: %f, Period: %f", time.seconds(), period.seconds());
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
        // RCLCPP_INFO(rclcpp::get_logger("ArmController"), "Got write update! Current time: %f, Period: %f", time.seconds(), period.seconds());
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

