#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <cstdint>
#include <stdint.h>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "arm_motor_controller/Communication/RS485.hpp"

namespace arm_motor_controller {

class Motor {
public:
    Motor(std::shared_ptr<RS485> rs485, uint32_t id, double startingPos = 0);

    void setJointLimits(double jointMin, double jointMax) {
        jointLimits[0] = jointMin;
        jointLimits[1] = jointMax;
    };

    void setMotorLimits(double motorMin, double motorMax) {
        motorLimits[0] = motorMin;
        motorLimits[1] = motorMax;
    };

    void setMotorSpeedScale(double speed) { scaledSpeed = speed; };

    virtual int enable();
    virtual int disable(bool isEmergency = false);

    virtual int exportState(std::vector<hardware_interface::StateInterface> &stateInterfaces, std::string jointName);
    virtual int exportCommand(std::vector<hardware_interface::CommandInterface> &commandInterfaces,
                              std::string jointName);

    virtual int read(double time, double period);
    virtual int write(double time, double period);

    double motorPos2Radians(double pos) {
        return (pos - motorLimits[0]) * ((jointLimits[1] - jointLimits[0]) / (motorLimits[1] - motorLimits[0])) +
               jointLimits[0];
    };
    double motorVel2Radians(double vel) { return vel / scaledSpeed; };

    double radians2MotorPos(double rad) {
        return (rad - jointLimits[0]) * ((motorLimits[1] - motorLimits[0]) / (jointLimits[1] - jointLimits[0])) +
               motorLimits[0];
    };
    double radians2MotorVel(double rad) { return rad * scaledSpeed; };

protected:
    std::shared_ptr<RS485> rs485;

    uint32_t id;

    double jointLimits[2] = {0.0, 1.0}; // ROS joint limits (min, max)
    double motorLimits[2] = {0.0, 1.0}; // Motor physical limits (min, max)
    double scaledSpeed = 1;

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