#include <string.h>

#include "arm_motor_controller/Communication/ModbusDevice.hpp"

namespace arm_motor_controller {

ModbusDevice::ModbusDevice(uint8_t id, RS485 &rs485, double decayRate)
    : deviceID(id), rs485(rs485), errorDecayRate(decayRate) {}

uint16_t ModbusDevice::crc16(const uint8_t *msg, uint8_t len) {
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

    uint16_t CRC2 = 0xFFFF;
    for (int crc16_i = 0; crc16_i < len; crc16_i++) {
        CRC2 ^= (*msg++) & 0x00FF;
        CRC2 = crc16_table[CRC2 & 0x00FF] ^ (CRC2 >> 8);
    }

    return CRC2;
}

int ModbusDevice::readRegisters(uint16_t reg, uint8_t func, uint16_t count, uint16_t *data) {
    uint8_t raw[ModbusDevice::MAX_PACKET_LEN] = {0};

    // build the packet
    raw[0] = deviceID;
    raw[1] = func;
    raw[2] = reg >> 8;
    raw[3] = reg;
    raw[4] = count >> 8;
    raw[5] = count;
    uint16_t crc = crc16(raw, 6);
    raw[6] = crc;
    raw[7] = crc >> 8;

    // Send downlink packet
    if (rs485.rawWrite(raw, 8) == -1) {
        writeErrorCounts++;
        return WRITE_ERROR; // Error writing
    }

    // memset(raw, 0, MAX_PACKET_LEN);

    uint16_t responseLen = (count + 3) * 2 - 1; // id + FC + #bytes + count*2 + crc16
    // wait for uplink packet response or timeout
    while (true) {
        // Check that the number of bytes read matches the requested count, timeout or error otherwise.
        int num = rs485.rawRead(raw, responseLen);
        if (num != responseLen) {
            readErrorCounts++;
            return READ_ERROR;
        }

        // Verify the correct packet was received.
        if (raw[0] == deviceID && raw[1] == func && raw[2] == count * 2 &&
            (raw[responseLen - 2] | (raw[responseLen - 1] << 8)) == crc16(raw, responseLen - 2)) {
            break;
        }
    }

    memcpy(data, &raw[3], count * 2);
    for (int i = 0; i < count; i++) {
        data[i] = __builtin_bswap16(data[i]);
    }

    return SUCCESS; // OK
}

int ModbusDevice::writeRegister(uint16_t reg, uint8_t func, uint16_t data) {
    uint8_t raw[ModbusDevice::MAX_PACKET_LEN] = {0};
    // RTUPacket raw;

    // build the packet
    raw[0] = deviceID;
    raw[1] = func;
    raw[2] = reg >> 8;
    raw[3] = reg;
    raw[4] = data >> 8;
    raw[5] = data;
    uint16_t crc = crc16(raw, 6);
    raw[6] = crc;
    raw[7] = crc >> 8;

    // Send downlink packet
    if (rs485.rawWrite(raw, 8) == -1) {
        writeErrorCounts++;
        return WRITE_ERROR; // Error writing
    }

    memset(raw, 0, MAX_PACKET_LEN);

    uint16_t responseLen = 8; // id + FC + #bytes + count*2 + crc16
    // wait for uplink packet response or timeout
    while (true) {
        // Check that the number of bytes read matches the requested count, timeout or error otherwise.
        int num = rs485.rawRead(raw, responseLen);
        if (num != responseLen) {
            // RCLCPP_INFO(rclcpp::get_logger("MotorState"), "Motor %d timeout, got %d bytes of %d, 0x%x", id, num,
            //             responseLen, raw[0]);
            readErrorCounts++;
            return READ_ERROR;
        }

        // Verify the correct packet was received.
        if (raw[0] == deviceID && raw[1] == func && raw[2] == reg >> 8 && raw[3] == (reg & 0xFF) &&
            (raw[responseLen - 2] | (raw[responseLen - 1] << 8)) == crc16(raw, responseLen - 2)) {
            break;
        }
    }

    return SUCCESS; // OK
}

int ModbusDevice::writeMultiple(uint16_t reg, uint8_t func, uint16_t count, const uint16_t *data) {
    uint8_t raw[ModbusDevice::MAX_PACKET_LEN] = {0};

    // build the packet
    raw[0] = deviceID;
    raw[1] = func;
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

    // Send downlink packet
    if (rs485.rawWrite(raw, 9 + count * 2) == -1) {
        writeErrorCounts++;
        return WRITE_ERROR; // Error writing
    }

    memset(raw, 0, MAX_PACKET_LEN);

    // wait for uplink packet response or timeout
    while (true) {
        // Check that the number of bytes read matches the requested count, timeout or error otherwise.
        int num = rs485.rawRead(raw, 8);
        if (num != 8) {
            readErrorCounts++;
            return READ_ERROR;
        }

        // Verify the correct packet was received.
        if (raw[0] == deviceID && raw[1] == func && raw[2] == reg >> 8 && raw[3] == (reg & 0xFF) &&
            (raw[6] | (raw[7] << 8)) == crc16(raw, 6)) {
            break;
        }
    }

    return SUCCESS; // OK
}

uint32_t ModbusDevice::update(double delta) {
    timeSinceLastDecrement += delta;

    while (timeSinceLastDecrement > errorDecayRate) {
        timeSinceLastDecrement -= errorDecayRate;
        if (writeErrorCounts > 0) {
            writeErrorCounts--;
        }
        if (readErrorCounts > 0) {
            readErrorCounts--;
        }
    }

    return readErrorCounts + writeErrorCounts;
}

} // namespace arm_motor_controller