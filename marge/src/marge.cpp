#include "marge/marge.hpp"

#include <iostream>

namespace marge {


    Marge::Marge(){};


    CallbackReturn Marge::on_init() {

// rclcpp::init();


        // Do some stuff
        std::cout << "HELLO WORLD, MARGE HAS BEEN INIT" << std::endl;

        
        // Make a node and cache it
        //  This is literally a RQT node, like a node node
        // FIXME this is a typing issue
        margeNode_ = std::make_shared<rclcpp::Node>(Marge());
        // RCLCPP_INFO(margeNode_.get_logger(), "Marge is on the couch!");
        


        //  need to spin, but spin makes it hang there, and we can't have that!
        // lambda thread yummy!
        nodeThread_ = std::thread([this](){
                rclcpp::spin(margeNode_);
        });
        // RCLCPP_INFO(margeNode_.get_logger(), "Node thread started!");


        // ROS node garbage here

        // This is what tutorial says to return but...
        // return controller_interface::return_type::OK; 

        // This is what the example code does
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Marge::on_deactivate(const rclcpp_lifecycle::State & previous_state){
        rclcpp::shutdown();
        nodeThread_.join();
        // RCLCPP_INFO(margeNode_.get_logger(), "Thread stopped and marge left"); 
    }


    


};
