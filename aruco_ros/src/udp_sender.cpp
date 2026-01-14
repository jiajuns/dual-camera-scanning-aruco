#include "aruco_ros/udp_sender.hpp"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <vector>
#include <cmath>

namespace aruco_ros
{

UdpSender::UdpSender(const std::string& ip, int port) : initialized_(false)
{
    if ((sockfd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        std::cerr << "Socket creation failed" << std::endl;
        return;
    }

    memset(&servaddr_, 0, sizeof(servaddr_));
    servaddr_.sin_family = AF_INET;
    servaddr_.sin_port = htons(port);
    
    if (inet_pton(AF_INET, ip.c_str(), &servaddr_.sin_addr) <= 0) {
        std::cerr << "Invalid address/ Address not supported" << std::endl;
        close(sockfd_);
        return;
    }

    initialized_ = true;
}

UdpSender::~UdpSender()
{
    if (initialized_) {
        close(sockfd_);
    }
}

// Helper to convert float to network byte order uint32_t
static uint32_t floatToNet(float f) {
    uint32_t i;
    std::memcpy(&i, &f, sizeof(float));
    return htonl(i);
}

void UdpSender::sendData(const std::vector<TagData>& tags, uint32_t seq, uint64_t timestamp_ns)
{
    if (!initialized_ || tags.empty()) return;

    std::vector<uint8_t> buffer;
    // Estimate size: Header (18) + Tags (32 * N)
    buffer.reserve(18 + tags.size() * 32);

    // 1. Magic (0xDEADBEEF) - 4 bytes
    uint32_t magic = htonl(0xDEADBEEF);
    const uint8_t* p = reinterpret_cast<const uint8_t*>(&magic);
    buffer.insert(buffer.end(), p, p + 4);

    // 2. Version (1) - 1 byte
    buffer.push_back(0x01);

    // 3. Count - 1 byte
    buffer.push_back(static_cast<uint8_t>(tags.size()));

    // 4. Sequence - 4 bytes
    uint32_t seq_net = htonl(seq);
    p = reinterpret_cast<const uint8_t*>(&seq_net);
    buffer.insert(buffer.end(), p, p + 4);

    // 5. Timestamp - 8 bytes (split into two 32-bit for network order if needed, or just copy if endianness matches or use simple approach)
    // Standard network order is big-endian.
    // Let's do manual big-endian packing for 64-bit
    uint64_t ts = timestamp_ns;
    for (int i = 7; i >= 0; --i) {
        buffer.push_back((ts >> (i * 8)) & 0xFF);
    }

    // 6. Payload
    for (const auto& tag : tags) {
        // ID - 4 bytes
        uint32_t id_net = htonl(static_cast<uint32_t>(tag.id));
        p = reinterpret_cast<const uint8_t*>(&id_net);
        buffer.insert(buffer.end(), p, p + 4);

        // Pos X, Y, Z
        uint32_t x = floatToNet(tag.pos.x);
        p = reinterpret_cast<const uint8_t*>(&x);
        buffer.insert(buffer.end(), p, p + 4);

        uint32_t y = floatToNet(tag.pos.y);
        p = reinterpret_cast<const uint8_t*>(&y);
        buffer.insert(buffer.end(), p, p + 4);

        uint32_t z = floatToNet(tag.pos.z);
        p = reinterpret_cast<const uint8_t*>(&z);
        buffer.insert(buffer.end(), p, p + 4);

        // Rot X, Y, Z, W
        uint32_t qx = floatToNet(static_cast<float>(tag.rot.x()));
        p = reinterpret_cast<const uint8_t*>(&qx);
        buffer.insert(buffer.end(), p, p + 4);

        uint32_t qy = floatToNet(static_cast<float>(tag.rot.y()));
        p = reinterpret_cast<const uint8_t*>(&qy);
        buffer.insert(buffer.end(), p, p + 4);

        uint32_t qz = floatToNet(static_cast<float>(tag.rot.z()));
        p = reinterpret_cast<const uint8_t*>(&qz);
        buffer.insert(buffer.end(), p, p + 4);

        uint32_t qw = floatToNet(static_cast<float>(tag.rot.w()));
        p = reinterpret_cast<const uint8_t*>(&qw);
        buffer.insert(buffer.end(), p, p + 4);
    }

    // Encode payload as ASCII hex (two chars per byte).
    std::string hex_payload;
    hex_payload.reserve(buffer.size() * 2);
    static const char kHexDigits[] = "0123456789ABCDEF";
    for (uint8_t b : buffer) {
        hex_payload.push_back(kHexDigits[(b >> 4) & 0x0F]);
        hex_payload.push_back(kHexDigits[b & 0x0F]);
    }

    ssize_t sent_bytes = sendto(sockfd_, hex_payload.data(), hex_payload.size(), 0, 
           (const struct sockaddr *)&servaddr_, sizeof(servaddr_));
    
    if (sent_bytes < 0) {
        std::cerr << "UDP sendto failed. Error: " << strerror(errno) << " (errno: " << errno << ")" << std::endl;
    } else if (static_cast<size_t>(sent_bytes) != hex_payload.size()) {
        std::cerr << "UDP sendto incomplete. Sent " << sent_bytes << " of " << hex_payload.size() << " bytes." << std::endl;
    }
}

} // namespace aruco_ros
