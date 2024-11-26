
#ifndef MARGE_CONTROLLER
#define MARGE_CONTROLLER

#include "controller_interface/controller_interface.hpp"

namespace marge {
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


    controller_interface::InterfaceConfiguration command_interface_configuration() const override;


    controller_interface::InterfaceConfiguration state_interface_configuration() const override;


    controller_interface::CallbackReturn on_init() override;

    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    // controller_interface::CallbackReturn update(const rclcpp_lifecycle::State & previous_state) override;
        


};

}  // namespace


#endif
