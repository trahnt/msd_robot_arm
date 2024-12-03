#ifndef MARGE_CONTROLLER
#define MARGE_CONTROLLER

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "controller_interface/controller_interface.hpp"

// using std::placeholders::_1;

namespace marge {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


using RefToLoanedState = std::reference_wrapper<hardware_interface::LoanedStateInterface>;
using RefToLoanedCommand = std::reference_wrapper<hardware_interface::LoanedCommandInterface>;



class Marge : public controller_interface::ControllerInterface {
public:

    // A LOT of other code uses these big all caps macro things
    // I belive they're for/from visibility_control.h which I am not doing
    // becuase it seems dumb
    //
    // Its for controlling visibility of the functions? but i don't care
    // Nothing matters anymore

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

    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

protected:
    std::vector<std::string> joint_names_;
    std::vector<std::string> command_interface_types_;
    std::vector<std::string> state_interface_types_;
    // The subscription to the /home_request ROS topic
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr home_message_subscriber_;


    // See the "using" statements at the top
    std::vector<RefToLoanedCommand> home_command_interface_;
    std::vector<RefToLoanedState> home_state_interface_;

    // String: vector of references to hardware loaned command interfaces
    std::unordered_map<std::string, std::vector<RefToLoanedCommand> *> command_interface_map_ = {{"home", &home_command_interface_}};
    std::unordered_map<std::string, std::vector<RefToLoanedState> *> state_interface_map_ = {{"home", &home_state_interface_}};


};
}

#endif
