#ifndef MODBUS_DEVICE_HPP
#define MODBUS_DEVICE_HPP

#include <stddef.h>
#include <stdint.h>
#include <string>

#include "RS485.hpp"

namespace arm_motor_controller {

class ModbusDevice {
public:
    /* Modbus constants */
    static constexpr int MAX_PACKET_LEN = 32;

    static constexpr int WRITE_ERROR = -1;
    static constexpr int READ_ERROR = -2;
    static constexpr int SUCCESS = 0;

    ModbusDevice(uint8_t id, RS485 &rs485, double decayRate = 1.0);
    ~ModbusDevice();

    /**
     * Calculates the 16-bit CRC checksum for a packet of data
     *
     * @param msg The array of the message bytes to calculate the CRC for
     * @param len The length of the message
     *
     * @returns The 16-bit CRC
     */
    uint16_t crc16(const uint8_t *msg, uint8_t len);

    /**
     * Reads from multiple registers on the device.
     *
     * @param reg The register to start reading from.
     * @param func The function code for the read.
     * @param count The number of registers to read from.
     * @param data A reference to data array to store result in.
     *
     * @returns 0 if success, error code otherwise.
     */
    int readRegisters(uint16_t reg, uint8_t func, uint16_t count, uint16_t *data);

    /**
     * Writes to a singe register on the device.
     *
     * @param reg The register to write to.
     * @param func The function code for the single write.
     * @param data The data to write.
     *
     * @returns 0 if success, error code otherwise
     */
    int writeRegister(uint16_t reg, uint8_t func, uint16_t data);

    /**
     * Writes to multiple continuous registers on the device.
     *
     * @param reg The register to start writing to.
     * @param func The function code for the multiple write.
     * @param count The number of registers to write to.
     * @param data A reference to data array to write to the registers.
     *
     * @returns 0 if success, error code otherwise.
     */
    int writeMultiple(uint16_t reg, uint8_t func, uint16_t count, const uint16_t *data);

    uint32_t update(double delta);

    void resetErrors() { readErrorCounts = writeErrorCounts = 0; };

    uint32_t getReadErrors() { return readErrorCounts; };
    uint32_t getWriteErrors() { return writeErrorCounts; };

private:
    // bool configure();

    /* Variables */
    uint8_t deviceID;
    RS485 &rs485;
    double errorDecayRate;

    double timeSinceLastDecrement = 0.0;
    uint32_t writeErrorCounts = 0;
    uint32_t readErrorCounts = 0;
};

} // namespace arm_motor_controller

#endif // MODBUS_DEVICE_HPP