#ifndef MARGE_CONTROLLER
#define MARGE_CONTROLLER

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "controller_interface/controller_interface.hpp"

using std::placeholders::_1;

namespace marge {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// The node that gets made inside the controller interface
class MargeNode : public rclcpp::Node{
public:
    MargeNode(): Node("Marge"){
        subscription_ = this->create_subscription<std_msgs::msg::Bool>(
            "home_request", 10, std::bind(&marge::MargeNode::topic_callback, this, _1));
    }

private:
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;

    void topic_callback(const std_msgs::msg::Bool::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(), "Marge is ready to control homer!");
    }
};




// The acutal controller
class Marge : public controller_interface::ControllerInterface {
public:

    // A LOT of other code uses these
    // I belive they're for/from visibility_control.h which I am not doing
    // becuase it seems dumb
    //
    // Its for controlling visibility of the functions? but i don't care
    // Nothing matters anymore, I'm gonna start pronouncing the L in salmon
    //
    // CONTROLLER_INTERFACE_PUBLIC
    Marge();

    // The node, so that we can actually do ROS stuff
    std::shared_ptr<rclcpp::Node>  margeNode_;
    // The thread that runs the node
    std::thread nodeThread_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr homingSubscriber_;

    CallbackReturn on_init() override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::return_type update( const rclcpp::Time & time, const rclcpp::Duration & period) override;




    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
};
}  // namespace


#endif
