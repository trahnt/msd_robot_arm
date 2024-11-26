#include "marge/marge.hpp"

#include <iostream>

namespace marge {

    Marge::Marge(){};


    controller_interface::CallbackReturn Marge::on_init(){
        // Do some stuff
        std::cout << "HELLO WORLD, MARGE HAS BEEN INIT" << std::endl;

        // ROS node garbage here

        // This is what tutorial says to return but...
        // return controller_interface::return_type::OK; 

        // This is what the example code does
        return CallbackReturn::SUCCESS;
    }
    


};
