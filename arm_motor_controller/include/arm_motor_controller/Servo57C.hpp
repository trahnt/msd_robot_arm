#include "arm_motor_controller/Motor.hpp"

namespace arm_motor_controller
{
    
class Servo57C : public Motor
{

public:
    Servo57C(uint32_t id, double jointMin, double jointMax, double motorMin, double motorMax, double startingPos = 0);
    ~Servo57C();

private:
    /* data */

};

} // namespace arm_motor_controller
