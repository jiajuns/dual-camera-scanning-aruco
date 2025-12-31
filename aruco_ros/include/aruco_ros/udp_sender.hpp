#ifndef ARUCO_ROS_UDP_SENDER_HPP
#define ARUCO_ROS_UDP_SENDER_HPP

#include <string>
#include <vector>
#include <netinet/in.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <opencv2/core.hpp>
#include <cstdint> 

namespace aruco_ros
{

struct TagData {
    int id;
    cv::Point3f pos;
    tf2::Quaternion rot;
};

class UdpSender
{
public:
    UdpSender(const std::string& ip, int port);
    ~UdpSender();

    void sendData(const std::vector<TagData>& tags, uint32_t seq, uint64_t timestamp_ns);

private:
    int sockfd_;
    struct sockaddr_in servaddr_;
    bool initialized_;
};

} // namespace aruco_ros

#endif // ARUCO_ROS_UDP_SENDER_HPP
