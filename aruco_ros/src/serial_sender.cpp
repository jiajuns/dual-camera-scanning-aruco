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

void SerialSender::sendData(const std::vector<TagData>& tags, uint32_t seq, uint64_t timestamp_ns)
{
    if (!initialized_) return;

    std::vector<uint8_t> buffer;
    buffer.reserve(61);  // 1 + 32 + 32 + 2(\r\n)
    
    // 第1字节：状态字节 (0=无tag, 1=1个tag, 2=2个tag)
    uint8_t status = static_cast<uint8_t>(std::min(tags.size(), size_t(2)));
    buffer.push_back(status);
    
    // 发送最多2个tag的数据
    for (size_t i = 0; i < 2; ++i) {
        if (i < tags.size()) {
            // 有效tag数据
            const auto& tag = tags[i];
            
            // ID (int32 -> 4 bytes)
            int32_t id = tag.id;
            const uint8_t* id_bytes = reinterpret_cast<const uint8_t*>(&id);
            buffer.push_back(static_cast<uint8_t>(tag.id));    

            // Position xyz (float32 -> 12 bytes)
            const uint8_t* x_bytes = reinterpret_cast<const uint8_t*>(&tag.pos.x);
            buffer.insert(buffer.end(), x_bytes, x_bytes + 4);
            
            const uint8_t* y_bytes = reinterpret_cast<const uint8_t*>(&tag.pos.y);
            buffer.insert(buffer.end(), y_bytes, y_bytes + 4);
            
            const uint8_t* z_bytes = reinterpret_cast<const uint8_t*>(&tag.pos.z);
            buffer.insert(buffer.end(), z_bytes, z_bytes + 4);
            
            // Quaternion qxyzw (float32 -> 16 bytes)
            float qx = static_cast<float>(tag.rot.x());
            float qy = static_cast<float>(tag.rot.y());
            float qz = static_cast<float>(tag.rot.z());
            float qw = static_cast<float>(tag.rot.w());
            
            const uint8_t* qx_bytes = reinterpret_cast<const uint8_t*>(&qx);
            buffer.insert(buffer.end(), qx_bytes, qx_bytes + 4);
            
            const uint8_t* qy_bytes = reinterpret_cast<const uint8_t*>(&qy);
            buffer.insert(buffer.end(), qy_bytes, qy_bytes + 4);
            
            const uint8_t* qz_bytes = reinterpret_cast<const uint8_t*>(&qz);
            buffer.insert(buffer.end(), qz_bytes, qz_bytes + 4);
            
            const uint8_t* qw_bytes = reinterpret_cast<const uint8_t*>(&qw);
            buffer.insert(buffer.end(), qw_bytes, qw_bytes + 4);
        } else {
            // 无效tag：填充255(id) + -500.0f(xyz和四元数)
            buffer.push_back(0xFF);  // ID = 255 表示无效
            
            // 填充-500.0f (7个float32 = 28字节)
            float invalid_value = -500.0f;
            const uint8_t* invalid_bytes = reinterpret_cast<const uint8_t*>(&invalid_value);
            
            for (int j = 0; j < 7; ++j) {  // x,y,z,qx,qy,qz,qw = 7个float
                buffer.insert(buffer.end(), invalid_bytes, invalid_bytes + 4);
            }
        }
    }
    
    // 添加换行符
    buffer.push_back('\r');
    buffer.push_back('\n');
    
    // 加入队列
    std::string msg(buffer.begin(), buffer.end());
    {
        std::lock_guard<std::mutex> lock(mutex_);
        
        const size_t MAX_QUEUE_SIZE = 2;
        while (queue_.size() >= MAX_QUEUE_SIZE) {
            queue_.pop();
        }
        
        queue_.push(std::move(msg));
    }
    cv_.notify_one();
}


} // namespace aruco_ros
