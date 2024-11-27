#include "rclcpp/rclcpp.hpp"

#include "arm_motor_controller/ICLStepper.hpp"

namespace arm_motor_controller {

ICLStepper::ICLStepper(std::shared_ptr<RS485> rs485, uint32_t id, double startingPos) : Motor(rs485, id, startingPos) {
    motorVel = 0;
    motorPos = startingPos;
}

ICLStepper::~ICLStepper() {}

uint16_t ICLStepper::crc16(const uint8_t *msg, uint8_t len) {
    static constexpr uint16_t crc16_table[256] = {
        0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241, 0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1,
        0xC481, 0x0440, 0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40, 0x0A00, 0xCAC1, 0xCB81, 0x0B40,
        0xC901, 0x09C0, 0x0880, 0xC841, 0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40, 0x1E00, 0xDEC1,
        0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41, 0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
        0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040, 0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1,
        0xF281, 0x3240, 0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441, 0x3C00, 0xFCC1, 0xFD81, 0x3D40,
        0xFF01, 0x3FC0, 0x3E80, 0xFE41, 0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840, 0x2800, 0xE8C1,
        0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41, 0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
        0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640, 0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0,
        0x2080, 0xE041, 0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240, 0x6600, 0xA6C1, 0xA781, 0x6740,
        0xA501, 0x65C0, 0x6480, 0xA441, 0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41, 0xAA01, 0x6AC0,
        0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840, 0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
        0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40, 0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1,
        0xB681, 0x7640, 0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041, 0x5000, 0x90C1, 0x9181, 0x5140,
        0x9301, 0x53C0, 0x5280, 0x9241, 0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440, 0x9C01, 0x5CC0,
        0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40, 0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
        0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40, 0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0,
        0x4C80, 0x8C41, 0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641, 0x8201, 0x42C0, 0x4380, 0x8341,
        0x4100, 0x81C1, 0x8081, 0x4040};

    uint16_t wCRC2 = 0xFFFF;
    for (int crc16_i = 0; crc16_i < len; crc16_i++) {
        wCRC2 ^= (*msg++) & 0x00FF;
        wCRC2 = crc16_table[wCRC2 & 0x00FF] ^ (wCRC2 >> 8);
    }

    return wCRC2;
}

int ICLStepper::configure() {
    constexpr uint16_t config[2][2] = {
        {0x0145, 0x26}, // DI1 mode (0x26 = neg sw, 0x25 = pos)
        {0x016D, 0x0},  // Alarm (0 = shut up)
    };

    writeRegister(config[0][0], config[0][1]);

    // uint16_t test[] = {0x0000, 0x0028, 0x0000, 0x0029};
    // RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Motor %d multi-write: %d", id, writeMultiple(0x0146, 4, test));

    return 0;
}

int ICLStepper::enable() {
    Motor::enable();

    writeRegister(ENABLE_REGISTER_ADDRESS_START, 0x01);

    return 0;
}

int ICLStepper::disable(bool isEmergency) {
    writeRegister(ENABLE_REGISTER_ADDRESS_START, 0x00);
    return 0;
}

int ICLStepper::readRegister(uint16_t reg, uint16_t count, uint16_t *data) {
    uint8_t raw[ICLStepper::MAX_PACKET_LEN] = {0};
    // RTUPacket raw;

    // build the packet
    raw[0] = id;
    raw[1] = READ_FUNCTION_CODE;
    raw[2] = reg >> 8;
    raw[3] = reg;
    raw[4] = count >> 8;
    raw[5] = count;
    uint16_t crc = crc16(raw, 6);
    raw[6] = crc;
    raw[7] = crc >> 8;

    // Send downlink packet
    if (rs485->rawWrite(raw, 8) == -1) {
        return -1; // Error writing
    }

    // memset(raw, 0, MAX_PACKET_LEN);

    uint16_t responseLen = (count + 3) * 2 - 1; // id + FC + #bytes + count*2 + crc16
    // wait for uplink packet response or timeout
    while (true) {
        // Check that the number of bytes read matches the requested count, timeout or error otherwise.
        int num = rs485->rawRead(raw, responseLen);
        if (num != responseLen) {
            // RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Motor %d timeout, got %d bytes of %d, 0x%x", id, num,
            //             responseLen, raw[0]);
            return -2;
        }

        // rcl_thr

        // std::stringstream ss;
        // ss << std::hex << std::setfill('0');
        // for (size_t i = 0; i < responseLen; ++i) {
        //     ss << std::setw(2) << static_cast<int>(raw[i]);
        // }
        // std::string str = ss.str();

        // RCLCPP_INFO(rclcpp::get_logger("MotorState"),
        //             "Motor %d response for %d registers starting at 0x%x; Request: %s", id, count, reg, str.c_str());

        // Verify the correct packet was received.
        if (raw[0] == id && raw[1] == READ_FUNCTION_CODE && raw[2] == count * 2 &&
            (raw[responseLen - 2] | (raw[responseLen - 1] << 8)) == crc16(raw, responseLen - 2)) {
            // RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Motor %d valid response", id);
            break;
        } // else {
        //     RCLCPP_INFO(rclcpp::get_logger("MotorState"),
        //                 "Motor %d invalid response: id? %d; FN? %d, LEN? %d, CRC? %d (%x=?%x)", id, raw[0] == id,
        //                 raw[1] == READ_FUNCTION_CODE, raw[2] == count * 2,
        //                 (raw[responseLen - 2] | (raw[responseLen - 1] << 8)) == crc16(raw, responseLen - 2),
        //                 raw[responseLen - 2] | (raw[responseLen - 1] << 8), crc16(raw, responseLen - 2));
        // }
    }

    memcpy(data, &raw[3], count * 2);
    for (int i = 0; i < count; i++) {
        data[i] = __builtin_bswap16(data[i]);
    }

    return 0; // OK
}

int ICLStepper::writeRegister(uint16_t reg, uint16_t data) {
    uint8_t raw[ICLStepper::MAX_PACKET_LEN] = {0};
    // RTUPacket raw;

    // build the packet
    raw[0] = id;
    raw[1] = WRITE_FUNCTION_CODE;
    raw[2] = reg >> 8;
    raw[3] = reg;
    raw[4] = data >> 8;
    raw[5] = data;
    uint16_t crc = crc16(raw, 6);
    raw[6] = crc;
    raw[7] = crc >> 8;

    // Send downlink packet
    if (rs485->rawWrite(raw, 8) == -1) {
        return -1; // Error writing
    }

    memset(raw, 0, MAX_PACKET_LEN);

    uint16_t responseLen = 8; // id + FC + #bytes + count*2 + crc16
    // wait for uplink packet response or timeout
    while (true) {
        // Check that the number of bytes read matches the requested count, timeout or error otherwise.
        int num = rs485->rawRead(raw, responseLen);
        if (num != responseLen) {
            // RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Motor %d timeout, got %d bytes of %d, 0x%x", id, num,
            //             responseLen, raw[0]);
            return -2;
        }

        // rcl_thr

        // std::stringstream ss;
        // ss << std::hex << std::setfill('0');
        // for (size_t i = 0; i < responseLen; ++i) {
        //     ss << std::setw(2) << static_cast<int>(raw[i]);
        // }
        // std::string str = ss.str();

        // RCLCPP_INFO(rclcpp::get_logger("MotorState"),
        //             "Motor %d response for %d registers starting at 0x%x; Request: %s", id, count, reg, str.c_str());

        // Verify the correct packet was received.
        if (raw[0] == id && raw[1] == WRITE_FUNCTION_CODE && raw[2] == reg >> 8 && raw[3] == (reg & 0xFF) &&
            (raw[responseLen - 2] | (raw[responseLen - 1] << 8)) == crc16(raw, responseLen - 2)) {
            // RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Motor %d valid response", id);
            break;
        } // else {
        //     RCLCPP_INFO(rclcpp::get_logger("MotorState"),
        //                 "Motor %d invalid response: id? %d; FN? %d, LEN? %d, CRC? %d (%x=?%x)", id, raw[0] == id,
        //                 raw[1] == READ_FUNCTION_CODE, raw[2] == count * 2,
        //                 (raw[responseLen - 2] | (raw[responseLen - 1] << 8)) == crc16(raw, responseLen - 2),
        //                 raw[responseLen - 2] | (raw[responseLen - 1] << 8), crc16(raw, responseLen - 2));
        // }
    }

    return 0; // OK
}

int ICLStepper::writeMultiple(uint16_t reg, uint16_t count, const uint16_t *data) {
    uint8_t raw[ICLStepper::MAX_PACKET_LEN] = {0};

    // build the packet
    raw[0] = id;
    raw[1] = WRITE_MULTIPLE_FUNCTION_CODE;
    raw[2] = reg >> 8; // starting register address
    raw[3] = reg;
    raw[4] = count >> 8; // register count
    raw[5] = count;
    raw[6] = (count * 2); // byte count

    // copy data bytes over
    for (int i = 0; i < count; i++) {
        raw[7 + 2 * i] = data[i] >> 8;
        raw[7 + 2 * i + 1] = data[i];
    }

    // calc crc
    uint16_t crc = crc16(raw, 7 + count * 2);
    raw[7 + count * 2] = crc;
    raw[7 + count * 2 + 1] = crc >> 8;

    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    for (size_t i = 0; i < (9 + count * 2); ++i) {
        ss << std::setw(2) << static_cast<int>(raw[i]);
    }
    std::string str = ss.str();

    RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Motor %d write for %d registers starting at 0x%x; Data: %s", id,
                count, reg, str.c_str());

    // Send downlink packet
    if (rs485->rawWrite(raw, 9 + count * 2) == -1) {
        return -1; // Error writing
    }

    memset(raw, 0, MAX_PACKET_LEN);

    // wait for uplink packet response or timeout
    while (true) {
        // Check that the number of bytes read matches the requested count, timeout or error otherwise.
        int num = rs485->rawRead(raw, 8);
        if (num != 8) {
            // RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Motor %d timeout, got %d bytes of %d, 0x%x", id, num,
            //             responseLen, raw[0]);
            return -2;
        }

        // rcl_thr

        // std::stringstream ss;
        // ss << std::hex << std::setfill('0');
        // for (size_t i = 0; i < responseLen; ++i) {
        //     ss << std::setw(2) << static_cast<int>(raw[i]);
        // }
        // std::string str = ss.str();

        // RCLCPP_INFO(rclcpp::get_logger("MotorState"),
        //             "Motor %d response for %d registers starting at 0x%x; Request: %s", id, count, reg, str.c_str());

        // Verify the correct packet was received.
        if (raw[0] == id && raw[1] == WRITE_MULTIPLE_FUNCTION_CODE && raw[2] == reg >> 8 && raw[3] == (reg & 0xFF) &&
            (raw[6] | (raw[7] << 8)) == crc16(raw, 6)) {
            // RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Motor %d valid response", id);
            break;
        } // else {
        //     RCLCPP_INFO(rclcpp::get_logger("MotorState"),
        //                 "Motor %d invalid response: id? %d; FN? %d, LEN? %d, CRC? %d (%x=?%x)", id, raw[0] == id,
        //                 raw[1] == READ_FUNCTION_CODE, raw[2] == count * 2,
        //                 (raw[responseLen - 2] | (raw[responseLen - 1] << 8)) == crc16(raw, responseLen - 2),
        //                 raw[responseLen - 2] | (raw[responseLen - 1] << 8), crc16(raw, responseLen - 2));
        // }
    }

    return 0; // OK
}

int ICLStepper::read(double time, double period) {
    (void)time, (void)period;
    // RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Motor %d read update", id);
    static int count = 0;

    if (count < 10) {
        count++;
        return 0;
    }

    count = 0;

    uint16_t raw[2] = {0};
    int ret = readRegister(ENCODER_REGISTER_ADDRESS_START, 2, raw);
    if (ret) {
        RCLCPP_WARN(rclcpp::get_logger("MotorState"), "Motor %d encountered and error while reading, got %d", id, ret);
        return -1;
    }

    double pos = static_cast<double>(raw[0] << 16 | raw[1]);

    rosCurrentPos = motorPos2Radians(pos);
    rosCurrentVel = motorVel2Radians(motorVel);

    // if (count > 30) {
    //     RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Motor %d pos: %0.4f", id, pos);
    //     count = 0;
    // }

    // count++;

    return 0;
}

int ICLStepper::write(double time, double period) {
    (void)time, (void)period;

    double pos = radians2MotorPos(rosTargetPos);
    double vel = radians2MotorVel(rosTargetVel);

    // // Only update the pos on change
    if (abs(pos - motorPos) < 1) {
        return 0;
    }

    RCLCPP_INFO(rclcpp::get_logger("MotorState"),
                "Motor %d update to position %0.4f (%0.4f) with velocity %0.4f (%0.4f)", id, pos, rosTargetPos, vel,
                rosTargetVel);

    int32_t realPos = pos;
    int16_t realVel = abs(vel) < 1.0 ? 1 : abs(vel);

    // Build the Immediate Trigger message
    uint16_t movementMessage[8] = {0};
    movementMessage[0] = PATH_MODE;
    movementMessage[1] = realPos >> 16;
    movementMessage[2] = realPos;
    movementMessage[3] = realVel;
    movementMessage[4] = ACCELERATION;
    movementMessage[5] = ACCELERATION;
    movementMessage[6] = PATH_DELAY;
    movementMessage[7] = PATH_TRIGGER;

    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    for (size_t i = 0; i < ARRAY_LEN(movementMessage); ++i) {
        ss << std::setw(2) << static_cast<int>(movementMessage[i]);
    }
    std::string str = ss.str();

    RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Motor %d moving to position %d with velocity %d for message %s", id,
                realPos, realVel, str.c_str());

    if (writeMultiple(PATH_REGISTER_ADDRESS_START, 8, movementMessage)) {
        RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Motor %d failed to move to position", id);
    }

    motorVel = vel;
    motorPos = pos;

    return 0;
}

} // namespace arm_motor_controller