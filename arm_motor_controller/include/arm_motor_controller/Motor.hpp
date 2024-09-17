#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <cstdint>
#include <stdint.h>

class Motor{
public:
    Motor();

    bool read();
    bool write();
    bool enable();
    bool disable(bool isEmergency = false);
protected:
    uint32_t id;
    int32_t pos;
    int32_t velocity;
    int32_t targetPos;
};

#endif //MOTOR_HPP