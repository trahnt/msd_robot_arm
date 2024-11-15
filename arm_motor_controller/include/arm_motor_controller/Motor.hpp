#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <cstdint>
#include <stdint.h>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace arm_motor_controller {

class Motor {
public:
    Motor(uint8_t id, double startingPos = 0);

    virtual int enable();
    virtual int disable(bool isEmergency = false);

    virtual int exportState(std::vector<hardware_interface::StateInterface> &stateInterfaces, std::string jointName);
    virtual int exportCommand(std::vector<hardware_interface::CommandInterface> &commandInterfaces,
                              std::string jointName);

    virtual int read(double time, double period);
    virtual int write(double time, double period);

protected:
    uint32_t id;
    double targetPos;
    double currentPos;

    double targetVel;
    double currentVel;

private:
    double lastupdate = 0.0;
};

} // namespace arm_motor_controller

#endif // MOTOR_HPP