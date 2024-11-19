#include "arm_motor_controller/Servo57C.hpp"

namespace arm_motor_controller {

Servo57C::Servo57C(uint32_t id, double jointMin, double jointMax, double motorMin, double motorMax, double startingPos) : Motor(id, jointMin, jointMax, motorMin, motorMax, startingPos)
{
}

Servo57C::~Servo57C()
{
}

}