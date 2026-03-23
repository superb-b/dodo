#ifndef CAN_PORT_H
#define CAN_PORT_H

#include <iostream>
#include <cstring>
#include <memory>
#include <cstdlib>
#include <cerrno>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>

class CanPort {
public:
    using SharedPtr = std::shared_ptr<CanPort>;

    explicit CanPort(const std::string& ifname = "can0") {
        struct ifreq ifr{};
        struct sockaddr_can addr{};

        // 打开 CAN socket
        socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socket_fd_ < 0) {
            perror("Error while opening CAN socket");
            std::exit(1);
        }

        // 设为 non-blocking，模仿官方 read_nonblocking 风格
        int flags = fcntl(socket_fd_, F_GETFL, 0);
        if (flags < 0) {
            perror("Error getting socket flags");
            close(socket_fd_);
            std::exit(1);
        }

        if (fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
            perror("Error setting CAN socket non-blocking");
            close(socket_fd_);
            std::exit(1);
        }

        // 可选：增大接收缓冲区，减少高频帧丢失风险
        int rcvbuf = 1024 * 1024;
        if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf)) < 0) {
            perror("Warning: failed to set SO_RCVBUF");
        }

        // 获取接口索引
        std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
        if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
            perror("Error getting interface index");
            close(socket_fd_);
            std::exit(1);
        }

        // 绑定到指定 CAN 接口
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
            perror("Error in CAN socket bind");
            close(socket_fd_);
            std::exit(1);
        }

        std::cout << "✅ Connected to CAN interface: " << ifname << std::endl;
    }

    ~CanPort() {
        if (socket_fd_ >= 0) {
            close(socket_fd_);
        }
    }

    // 发送 CAN 帧
    ssize_t send(uint32_t can_id, const uint8_t* data, uint8_t len) {
        struct can_frame frame{};
        frame.can_id = can_id & CAN_SFF_MASK;
        frame.can_dlc = len;
        std::memcpy(frame.data, data, len);
        return write(socket_fd_, &frame, sizeof(frame));
    }

    // non-blocking recv:
    // 有数据 -> 返回读到的字节数
    // 没数据 -> 返回 0
    // 出错   -> 返回 -1
    ssize_t recv(uint32_t& can_id, uint8_t* data, uint8_t& len) {
        struct can_frame frame{};
        const ssize_t nbytes = read(socket_fd_, &frame, sizeof(frame));

        if (nbytes < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return 0;  // 当前没有数据
            }
            return -1;     // 真正错误
        }

        if (nbytes == 0) {
            return 0;
        }

        can_id = frame.can_id & CAN_SFF_MASK;
        len = frame.can_dlc;
        std::memcpy(data, frame.data, len);
        return nbytes;
    }

private:
    int socket_fd_{-1};
};

#endif // CAN_PORT_H