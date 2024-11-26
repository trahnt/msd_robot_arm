
#ifndef MARGE_CONTROLLER
#define MARGE_CONTROLLER

#include "controller_interface/controller_interface.hpp"

namespace marge {
class Marge : public controller_interface::ControllerInterface {
public:

    Marge();
    ~Marge();

    /*
        command_interface_configuration
        state_insterface_configuration
        on_configure
        on_activate
        on_deactivate
        update
    */
        


};

}  // namespace


#endif
