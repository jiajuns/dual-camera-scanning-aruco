#include <rclcpp/rclcpp.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <string>
#include <cerrno>
class SerialTestNode : public rclcpp::Node
{
public:
    SerialTestNode() : Node("serial_test_node")
    {
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud_rate", 115200);

        std::string port;
        int baud;
        this->get_parameter("serial_port", port);
        this->get_parameter("baud_rate", baud);

        fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd_ == -1) {
        //rerurn the reason
            RCLCPP_ERROR(this->get_logger(), 
                         "无法打开串口 %s. 错误原因: %s (错误代码: %d)", 
                         port.c_str(), strerror(errno), errno);
            return;
        }

        // Restore blocking behavior or set non-blocking
        fcntl(fd_, F_SETFL, 0); 

        struct termios options;
        tcgetattr(fd_, &options);

        speed_t baud_speed;
        switch (baud) {
            case 9600: baud_speed = B9600; break;
            case 19200: baud_speed = B19200; break;
            case 38400: baud_speed = B38400; break;
            case 57600: baud_speed = B57600; break;
            case 115200: baud_speed = B115200; break;
            case 230400: baud_speed = B230400; break;
            case 460800: baud_speed = B460800; break;
            case 921600: baud_speed = B921600; break;
            default: baud_speed = B115200; break;
        }

        cfsetispeed(&options, baud_speed);
        cfsetospeed(&options, baud_speed);

        // 8N1
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_cflag |= (CLOCAL | CREAD);
        
        // Raw mode
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_oflag &= ~OPOST;

        tcsetattr(fd_, TCSANOW, &options);

        RCLCPP_INFO(this->get_logger(), "Serial port %s opened at %d baud", port.c_str(), baud);

        // Timer to send data every 1 second
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&SerialTestNode::timerCallback, this));
    }

    ~SerialTestNode()
    {
        if (fd_ != -1) close(fd_);
    }

private:
    void timerCallback()
    {
        if (fd_ == -1) return;
        
        // 修改点：将 "\n" 改为 "\r\n"
        // 这样才能匹配 STM32 usart.c 中判断 (0x0d 0x0a) 的逻辑
        std::string msg = "1234567890\r\n";
        
        ssize_t written = write(fd_, msg.c_str(), msg.length());
        if (written < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to write to serial port: %s", strerror(errno));
        } else {
            // 打印到终端时去掉结尾的 \r\n 方便观察
            RCLCPP_INFO(this->get_logger(), "Sent: %s", msg.substr(0, msg.length()-2).c_str());
        }
    }

    int fd_ = -1;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialTestNode>());
    rclcpp::shutdown();
    return 0;
}
