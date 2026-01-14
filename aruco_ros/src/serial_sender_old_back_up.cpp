#include "aruco_ros/serial_sender.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <arpa/inet.h>

namespace aruco_ros
{

SerialSender::SerialSender(const std::string& port, int baud_rate) : initialized_(false), running_(true)
{
    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ == -1) {
        std::cerr << "Unable to open serial port " << port << std::endl;
        // Even if failed, we start the thread to avoid crash in destructor join
        sender_thread_ = std::thread(&SerialSender::senderLoop, this);
        return;
    }

    // Non-blocking access
    fcntl(fd_, F_SETFL, FNDELAY);

    struct termios options;
    tcgetattr(fd_, &options);

    speed_t baud;
    switch (baud_rate) {
        case 9600: baud = B9600; break;
        case 19200: baud = B19200; break;
        case 38400: baud = B38400; break;
        case 57600: baud = B57600; break;
        case 115200: baud = B115200; break;
        case 230400: baud = B230400; break;
        case 460800: baud = B460800; break;
        case 921600: baud = B921600; break;
        default: baud = B115200; break;
    }

    cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);

    // 8N1
    options.c_cflag &= ~PARENB; // No parity
    options.c_cflag &= ~CSTOPB; // 1 stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;     // 8 data bits

    options.c_cflag |= (CLOCAL | CREAD); // Enable receiver, ignore modem control lines
    
    // Raw input/output
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;

    tcsetattr(fd_, TCSANOW, &options);

    initialized_ = true;
    sender_thread_ = std::thread(&SerialSender::senderLoop, this);
}

SerialSender::~SerialSender()
{
    running_ = false;
    cv_.notify_all();
    if (sender_thread_.joinable()) {
        sender_thread_.join();
    }

    if (initialized_) {
        close(fd_);
    }
}

void SerialSender::senderLoop()
{
    while (running_) {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] { return !queue_.empty() || !running_; });

        while (!queue_.empty()) {
            std::string msg = std::move(queue_.front());
            queue_.pop();
            lock.unlock();

            if (initialized_) {
                write(fd_, msg.c_str(), msg.length());
            }

            lock.lock();
        }
    }
}

static int8_t clampToInt8(float value, float scale) {
    float scaled = value * scale;
    if (scaled > 127.0f) return 127;
    if (scaled < -127.0f) return -127;
    return static_cast<int8_t>(std::round(scaled));
}

void SerialSender::sendData(const std::vector<TagData>& tags, uint32_t seq, uint64_t timestamp_ns)
{
    if (!initialized_) return;

    std::vector<uint8_t> buffer;
    
    // 识别失败：0个tag或超过2个tag
    if (tags.empty() || tags.size() > 2) {
        // 发送失败标记：0xFF + 7个0x00
        buffer.push_back(0xFF);
        for (int i = 0; i < 7; ++i) {
            buffer.push_back(0x00);
        }
    } else {
        // 识别成功：1-2个tag
        for (const auto& tag : tags) {
            // ID (1字节)
            buffer.push_back(static_cast<uint8_t>(tag.id));
            
            // 位置 xyz (各1字节，单位：厘米)
            int8_t x_cm = clampToInt8(tag.pos.x, 100.0f);
            int8_t y_cm = clampToInt8(tag.pos.y, 100.0f);
            int8_t z_cm = clampToInt8(tag.pos.z, 100.0f);
            buffer.push_back(static_cast<uint8_t>(x_cm));
            buffer.push_back(static_cast<uint8_t>(y_cm));
            buffer.push_back(static_cast<uint8_t>(z_cm));
            
            // 四元数 qx qy qz qw (各1字节，×127)
            int8_t qx = clampToInt8(static_cast<float>(tag.rot.x()), 127.0f);
            int8_t qy = clampToInt8(static_cast<float>(tag.rot.y()), 127.0f);
            int8_t qz = clampToInt8(static_cast<float>(tag.rot.z()), 127.0f);
            int8_t qw = clampToInt8(static_cast<float>(tag.rot.w()), 127.0f);
            buffer.push_back(static_cast<uint8_t>(qx));
            buffer.push_back(static_cast<uint8_t>(qy));
            buffer.push_back(static_cast<uint8_t>(qz));
            buffer.push_back(static_cast<uint8_t>(qw));
        }
    }
    
    // 添加换行符
    buffer.push_back('\r');
    buffer.push_back('\n');
    
    // 转换为string并加入队列
    std::string msg(buffer.begin(), buffer.end());
    {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push(std::move(msg));
    }
    cv_.notify_one();
}

} // namespace aruco_ros

