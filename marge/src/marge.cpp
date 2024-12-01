#include "marge/marge.hpp"

// #include <iostream>


using config_type = controller_interface::interface_configuration_type;

namespace marge {


    Marge::Marge(){};


    CallbackReturn Marge::on_init() {
        // should have error handling
        joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
        command_interface_types_ =
            auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
        state_interface_types_ =
            auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

        //point_interp_.positions.assign(joint_names_.size(), 0);
        //point_interp_.velocities.assign(joint_names_.size(), 0);

        // This is what the example code does
        RCLCPP_INFO(this->get_node()->get_logger(), "Marge is on the couch");
        //  RCLCPP_INFO(this->get_node()->get_logger(), );
        return CallbackReturn::SUCCESS;
    }


    controller_interface::CallbackReturn Marge::on_configure(const rclcpp_lifecycle::State &) {
        // our callback lambda for what to do when we get a message
        auto callback = [this](const std::shared_ptr<std_msgs::msg::Bool> bool_msg) -> void {
            RCLCPP_INFO(this->get_node()->get_logger(), "Marge got the home message!");
            
            // TODO matt
            // This is where the homing algirhtm stuff will go.
            // When marge gets a home_request message, this function gets ran


            // from example7
            // traj_msg_external_point_ptr_.writeFromNonRT(traj_msg);
            // new_msg_ = true;
        };

        home_message_subscriber_ = get_node()->create_subscription<std_msgs::msg::Bool>(
        "/home_request", rclcpp::SystemDefaultsQoS(), callback);

        return CallbackReturn::SUCCESS;
    }



    controller_interface::CallbackReturn Marge::on_deactivate(const rclcpp_lifecycle::State & previous_state){
        // rclcpp::shutdown();
        // nodeThread_.join();
        RCLCPP_INFO(this->get_node()->get_logger(), "MARGE DEACTIVATED"); 
        return CallbackReturn::SUCCESS;
    }
 
    controller_interface::InterfaceConfiguration Marge::command_interface_configuration() const {
        //return CallbackReturn::SUCCESS;
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        // Copied from example7
        
        // NOT GONNA LIE this kinda just... worked
        // first ever for ROS 
        conf.names.reserve(joint_names_.size() * state_interface_types_.size());
        for (const auto & joint_name : joint_names_) {
            for (const auto & interface_type : state_interface_types_) {
              conf.names.push_back(joint_name + "/" + interface_type);
            }
        }

        return conf;
    }

    controller_interface::InterfaceConfiguration Marge::state_interface_configuration() const {
    
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        // FIXME
        // This is probably where we read from the yaml, but I'm not gonna worry
        // about htat *yet*
        conf.names.reserve(joint_names_.size() * state_interface_types_.size());
        for (const auto & joint_name : joint_names_) {
            for (const auto & interface_type : state_interface_types_)
            {
              conf.names.push_back(joint_name + "/" + interface_type);
            }
        }

        return conf;
    }

    controller_interface::return_type Marge::update(const rclcpp::Time & time, const rclcpp::Duration & period){
        // this is what runs continuously over and over I think
        return controller_interface::return_type::OK;
    }
};


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(marge::Marge, controller_interface::ControllerInterface)
