#ifndef RS485_HPP
#define RS485_HPP

#include <stddef.h>
#include <stdint.h>
#include <string>

namespace arm_motor_controller {

enum class RS485Parity { None, Even, Odd };

class RS485 {
public:
    RS485(const char *portName, int baudrate, RS485Parity parity, uint8_t byteSize, uint8_t stopBits, uint8_t timeout);
    ~RS485();

    /**
     * Connect to the Serial port
     */
    bool connect();

    /**
     * Disconnect from the Serial port
     */
    int disconnect();

    bool isConnected() { return connected; };

    /**
     * Writes data to the RS485 bus
     */
    int rawWrite(uint8_t *buf, size_t size);

    /**
     * Reads data from the RS485 bus
     */
    int rawRead(uint8_t *buf, size_t size);

private:
    bool configure();

    // Config settings:
    std::string portName;
    int baudrate;
    RS485Parity parity;
    uint8_t byteSize;
    uint8_t stopBits;
    uint8_t timeout;

    // States
    bool connected;
    int fd;
};

} // namespace arm_motor_controller

#endif // RS485_HPP