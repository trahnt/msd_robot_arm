#include "marge/marge.hpp"

// #include <iostream>

namespace marge {


    Marge::Marge(){ 
        // std::cout << "CONSTRUCTOR" << std::endl;
    };


    CallbackReturn Marge::on_init() {

        RCLCPP_INFO(this->get_node()->get_logger(), "CONSTRUCTOR");

        // std::cout << "ON INIT" << std::endl;
        //RCLCPP_INFO(this->get_node()->get_logger(), "ON INIT");
 
        //  need to spin, but spin makes it hang there, and we can't have that!
        // lambda thread yummy!
        
        /*
        nodeThread_ = std::thread([this](){
                rclcpp::spin(margeNode_);
        });
        */

        // RCLCPP_INFO(margeNode_.get_logger(), "Node thread started!");

        // ROS node garbage here

        // This is what tutorial says to return but...
        // return controller_interface::return_type::OK; 

        // This is what the example code does
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Marge::on_deactivate(const rclcpp_lifecycle::State & previous_state){
        // rclcpp::shutdown();
        // nodeThread_.join();
        // RCLCPP_INFO(margeNode_.get_logger(), "Thread stopped and marge left"); 
    }

    
    controller_interface::InterfaceConfiguration Marge::command_interface_configuration() const {}
    controller_interface::InterfaceConfiguration Marge::state_interface_configuration() const {}
    controller_interface::return_type Marge::update(const rclcpp::Time & time, const rclcpp::Duration & period){}




};

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(marge::Marge, controller_interface::ControllerInterface)
