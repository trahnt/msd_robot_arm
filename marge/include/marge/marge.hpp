#ifndef MARGE_CONTROLLER
#define MARGE_CONTROLLER

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "controller_interface/controller_interface.hpp"

// using std::placeholders::_1;

namespace marge {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// The node that gets made inside the controller interface





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
    // std::shared_ptr<rclcpp::Node>  margeNode_;
    // The thread that runs the node
    // std::thread nodeThread_;

    CallbackReturn on_init() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::return_type update( const rclcpp::Time & time, const rclcpp::Duration & period) override;

    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

protected:
    std::vector<std::string> joint_names_;
    std::vector<std::string> command_interface_types_;
    std::vector<std::string> state_interface_types_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr home_message_subscriber_;

};
}  // namespace


#endif
