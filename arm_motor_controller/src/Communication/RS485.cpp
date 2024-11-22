#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

#include "arm_motor_controller/Communication/RS485.hpp"

namespace arm_motor_controller {

RS485::RS485(const char *portName, int baudrate, RS485Parity parity, uint8_t byteSize, uint8_t stopBits,
             uint8_t timeout)
    : portName(portName), baudrate(baudrate), parity(parity), byteSize(byteSize), stopBits(stopBits), timeout(timeout) {

}

RS485::~RS485() { disconnect(); }

bool RS485::connect() {
    connected = false;
    fd = open(portName.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        RCLCPP_FATAL(rclcpp::get_logger("RS485"), "Error %s, Failed to open \"%s\"", strerror(errno), portName.c_str());
        return false;
    }
    connected = configure();

    RCLCPP_INFO(rclcpp::get_logger("RS485"), "Comms started");

    return connected;
}

bool RS485::configure() {
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        // cerr << "Error from tcgetattr: " << strerror(errno) << endl;
        return false;
    }

    // Baudrate
    int speedOption = B9600;
    switch (baudrate) {
    case 9600:
        break;
    case 19200:
        speedOption = B19200;
        break;
    case 38400:
        speedOption = B38400;
        break;
    case 57600:
        speedOption = B57600;
        break;
    case 115200:
        speedOption = B115200;
        break;

    default:
        RCLCPP_FATAL(rclcpp::get_logger("RS485"), "Invalid baudrate '%d'", baudrate);
        return false;
    }

    // Parity
    unsigned int parityOption = 0;
    switch (parity) {
    case RS485Parity::None:
        break;
    case RS485Parity::Even:
        parityOption = (PARENB);
        break;
    case RS485Parity::Odd:
        parityOption = (PARENB | PARODD);
        break;
    default:
        return false;
    }

    // byte size
    unsigned int byteSizeOption = CS5;
    switch (byteSize) {
    case 5:
        break;
    case 6:
        byteSizeOption = CS6;
        break;
    case 7:
        byteSizeOption = CS7;
        break;
    case 8:
        byteSizeOption = CS8;
        break;

    default:
        RCLCPP_FATAL(rclcpp::get_logger("RS485"), "Invalid byte size '%d'", byteSize);
        return false;
    }

    // stop bits
    unsigned int stopBitsOption = stopBits == 2 ? CSTOPB : ~CSTOPB;

    cfsetospeed(&tty, speedOption);
    cfsetispeed(&tty, speedOption);

    tty.c_iflag &= ~IGNBRK;          // disable break processing
    tty.c_lflag = 0;                 // no signaling chars, no echo, no, canonical processing
    tty.c_oflag = 0;                 // no remapping, no delays
    tty.c_cc[VMIN] = 0;              // read doesn't block
    tty.c_cc[VTIME] = timeout / 100; // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | byteSizeOption; // set byte size
    tty.c_cflag &= ~(PARENB | PARODD);                     // parity clear
    tty.c_cflag |= parityOption;                           // parity set
    tty.c_cflag &= stopBitsOption;                         // stop bits
    tty.c_cflag |= (CLOCAL | CREAD);                       // ignore modem controls, enable reading
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        // cerr << "Error from tcsetattr: " << strerror(errno) << endl;
        return false;
    }
    return true;
}

int RS485::disconnect() {
    connected = false;
    RCLCPP_INFO(rclcpp::get_logger("RS485"), "Comms stopped");
    return close(fd);
}

int RS485::rawWrite(uint8_t *buf, size_t size) {
    int ret = write(fd, buf, size);
    tcdrain(fd); // wait for send to finish
    return ret;
}

int RS485::rawRead(uint8_t *buf, size_t size) { return read(fd, buf, size); }

} // namespace arm_motor_controller