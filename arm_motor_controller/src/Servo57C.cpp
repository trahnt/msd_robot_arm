#include "rclcpp/rclcpp.hpp"

#include "arm_motor_controller/Servo57C.hpp"

namespace arm_motor_controller {

namespace {
#define ARRAY_LEN(arr) (sizeof(arr) / sizeof(arr[0]))

/**
 * Message Struct for sending a data to the Servo57C. Uses a funny layout to easily be able to pack data in by name and
 * access it as a raw array.
 */
#pragma pack(1) // Pack structures on 1-byte boundaries
union PositionCommand {
    // True message with array accesss
    struct {
        // Message structure
        uint8_t head;
        uint8_t addr;
        uint8_t func;
        // byte 4
        struct {
            uint8_t speedH : 4;
            uint8_t _reserved : 3;
            uint8_t dir : 1;
        };
        // byte 5
        uint8_t speedL : 8;
        // byte 6
        uint8_t accel : 8;
        // byte 7-10
        uint32_t pulses : 32;

        uint8_t crc;
    };

    // Array accesss
    uint8_t raw[11] = {0};
};

/**
 * Struct for holding and parsing encoder position data
 */
union EncoderData {
    struct {
        int32_t carry;
        uint16_t val;
    };
    uint8_t raw[6];
};
#pragma pack()

} // namespace

Servo57C::Servo57C(std::shared_ptr<RS485> rs485, uint32_t id, double startingPos) : Motor(rs485, id, startingPos) {
    motorVel = 0;
    motorPos = startingPos;
}

Servo57C::~Servo57C() {}

uint8_t Servo57C::crc8(uint8_t *msg, uint8_t len) {
    int crc = 0;
    for (int i = 0; i < len; i++) {
        crc += msg[i];
    }
    return crc & 0xFF;
}

int Servo57C::readParameter(uint8_t func, uint8_t *data, uint8_t dataLen) {
    uint8_t raw[Servo57C::MAX_PACKET_LEN] = {0};

    // build the packet
    raw[0] = DOWNLINK_PACKET_HEAD;
    raw[1] = id;
    raw[2] = func;
    raw[3] = crc8(raw, 3);

    // Send downlink packet
    if (rs485->rawWrite(raw, 4) == -1) {
        return -1; // Error writing
    }

    // wait for uplink packet response or timeout
    while (true) {
        // Check that the number of bytes read matches the requested count, timeout or error otherwise.
        if (rs485->rawRead(raw, 4 + dataLen) != 4 + dataLen) {
            return -2;
        }

        // Verify the correct packet was received.
        if (raw[0] == UPLINK_PACKET_HEAD && raw[1] == id && raw[2] == func &&
            raw[dataLen + 3] == crc8(raw, dataLen + 3)) {
            break;
        }
    }

    memcpy(data, &raw[3], dataLen);
    return 0; // OK
}

int Servo57C::writeParameter(uint8_t func, uint8_t &data) {
    uint8_t raw[Servo57C::MAX_PACKET_LEN] = {0};

    // build the packet
    raw[0] = DOWNLINK_PACKET_HEAD;
    raw[1] = id;
    raw[2] = func;
    raw[3] = data;
    raw[4] = crc8(raw, 4);

    // Send downlink packet
    if (rs485->rawWrite(raw, 5) == -1) {
        return -1; // Error writing
    }

    // wait for uplink packet response or timeout
    while (true) {
        // Check that the number of bytes read matches the requested count, timeout or error otherwise.
        if (rs485->rawRead(raw, 5) != 5) {
            return -2;
        }

        // Verify the correct packet was received.
        if (raw[0] == UPLINK_PACKET_HEAD && raw[1] == id && raw[2] == func && raw[4] == crc8(raw, 4)) {
            break;
        }
    }

    data = raw[3];
    return 0; // OK
}

int Servo57C::read(double time, double period) {
    (void)time, (void)period;
    // RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Motor %d read update", id);

    EncoderData rawPos;
    int ret = readParameter(0x30, rawPos.raw, 6);
    if (ret) {
        // FIXME I'M IGNORING ERRORS
        // RCLCPP_WARN(rclcpp::get_logger("MotorState"), "Motor %d encountered and error while reading, got %d", id, ret);
        // return -1;
        return 0;
    }

    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    for (size_t i = 0; i < ARRAY_LEN(rawPos.raw); ++i) {
        ss << std::setw(2) << static_cast<int>(rawPos.raw[i]);
    }
    std::string str = ss.str();

    // fix endianness
    rawPos.carry = __builtin_bswap32(rawPos.carry);
    rawPos.val = __builtin_bswap16(rawPos.val);

    double pos =
        ((static_cast<double>(rawPos.carry) * ENCODER_VALUE_MAX + static_cast<double>(rawPos.val)) * PULSES_PER_REV) /
        ENCODER_VALUE_MAX;

    rosCurrentPos = motorPos2Radians(pos);
    rosCurrentVel = motorVel2Radians(motorVel);

    // RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Motor %d position: %.4f (%.4f) [%x, %x]; raw: %s", id,
    // rosCurrentPos,
    //             pos, rawPos.carry, rawPos.val, str.c_str());

    return 0;
}

int Servo57C::write(double time, double period) {
    (void)time, (void)period;

    double pos = radians2MotorPos(rosTargetPos);
    double vel = radians2MotorVel(rosTargetVel);

    // Only update the pos on change
    if (abs(pos - motorPos) < 1) {
        return 0;
    }

    // RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Motor %d target pos %.4f (%.4f) with vel %.4f (%.4f)", id,
    //             rosTargetPos, pos, rosTargetVel, vel);

    // int32_t relPos = pos - motorPos;
    bool isForward = vel > 0.0; // || relPos < 0;

    int32_t relPos = abs(pos - motorPos);
    int16_t relVel = abs(vel) < 1.0 ? 1 : abs(vel);

    // Build message
    PositionCommand msg;
    msg.head = 0xFA;
    msg.addr = static_cast<uint8_t>(id);
    msg.func = 0xFD;
    msg.dir = isForward;
    msg.speedH = static_cast<uint8_t>((relVel >> 8) & 0xF);
    msg.speedL = static_cast<uint8_t>(relVel & 0xFF);
    msg.accel = ACCELERATION;
    msg.pulses = __builtin_bswap32(static_cast<uint32_t>(relPos)); // uint32 gets packed in the wrong order so swap
                                                                   // bytes

    // Calc crc8 and place it at the end of the message
    msg.crc = crc8(msg.raw, ARRAY_LEN(msg.raw));

    // RCLCPP_INFO(rclcpp::get_logger("MotorState"),
    //             "Motor %d head: %x, addr: %x, func: %x, dir: %d, speed: %x%x, accel: %x, pulse: %d", id, msg.head,
    //             msg.addr, msg.func, msg.dir, msg.speedH, msg.speedL, msg.accel, __builtin_bswap32(msg.pulses));

    rs485->rawWrite(msg.raw, 11);

    motorVel = vel;
    motorPos = pos;

    return 0;
}

} // namespace arm_motor_controller
