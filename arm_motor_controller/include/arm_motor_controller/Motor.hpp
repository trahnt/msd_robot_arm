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
    Motor(uint32_t id, double jointMin, double jointMax, double motorMin, double motorMax, double startingPos = 0);

    virtual int enable();
    virtual int disable(bool isEmergency = false);

    virtual int exportState(std::vector<hardware_interface::StateInterface> &stateInterfaces, std::string jointName);
    virtual int exportCommand(std::vector<hardware_interface::CommandInterface> &commandInterfaces,
                              std::string jointName);

    virtual int read(double time, double period);
    virtual int write(double time, double period);

    double motorPos2Radians(double pos) {return (pos-motorLimits[0]) * ((jointLimits[1]-jointLimits[0])/(motorLimits[1]-motorLimits[0])) + jointLimits[0];};
    double motorVel2Radians(double pos) { return pos; };

    double radians2MotorPos(double rad) {return (rad-jointLimits[0]) * ((motorLimits[1]-motorLimits[0])/(jointLimits[1]-jointLimits[0])) + motorLimits[0]; };
    double radians2MotorVel(double rad) { return rad; };

protected:
    uint32_t id;

    double jointLimits[2]; // ROS joint limits (min, max)
    double motorLimits[2]; // Motor physical limits (min, max)

    double rosCurrentPos;
    double rosTargetPos;

    double rosCurrentVel;
    double rosTargetVel;

    double motorPos;
    double motorVel;

private:
    double lastupdate = 0.0;
};

} // namespace arm_motor_controller

#endif // MOTOR_HPP