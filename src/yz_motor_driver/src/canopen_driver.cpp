#include "yz_motor_driver/canopen_driver.hpp"
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <sys/select.h>
#include <iomanip>
#include <cmath>

namespace yz_motor_driver {

CANopenDriver::CANopenDriver(const std::string& interface, uint8_t node_id)
    : interface_(interface), node_id_(node_id) {
}

CANopenDriver::~CANopenDriver() {
    running_ = false;
    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }

    if (can_socket_ >= 0) {
        close(can_socket_);
    }
}

bool CANopenDriver::init() {
    struct sockaddr_can addr;
    struct ifreq ifr;

    // 创建套接字
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
        std::cerr << "Error creating socket: " << strerror(errno) << std::endl;
        return false;
    }

    // 获取接口索引
    strcpy(ifr.ifr_name, interface_.c_str());
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
        std::cerr << "Error getting interface index: " << strerror(errno) << std::endl;
        close(can_socket_);
        can_socket_ = -1;
        return false;
    }

    // 绑定套接字
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        std::cerr << "Error binding socket: " << strerror(errno) << std::endl;
        close(can_socket_);
        can_socket_ = -1;
        return false;
    }

    // 启动接收线程
    running_ = true;
    receive_thread_ = std::thread(&CANopenDriver::receiveThread, this);

    std::cout << "CANopen driver initialized for node " << static_cast<int>(node_id_) << " on interface " << interface_ << std::endl;
    return true;
}

bool CANopenDriver::resetNode() {
    std::vector<uint8_t> data = {0x81, node_id_, 0, 0, 0, 0, 0, 0};
    return sendFrame(0, data);  // NMT消息的COB-ID是0
}

bool CANopenDriver::setPreOperational() {
    std::vector<uint8_t> data = {0x80, node_id_, 0, 0, 0, 0, 0, 0};
    return sendFrame(0, data);  // NMT消息的COB-ID是0
}

bool CANopenDriver::startRemoteNode() {
    std::vector<uint8_t> data = {0x01, node_id_, 0, 0, 0, 0, 0, 0};
    return sendFrame(0, data);  // NMT消息的COB-ID是0
}

bool CANopenDriver::stopRemoteNode() {
    std::vector<uint8_t> data = {0x02, node_id_, 0, 0, 0, 0, 0, 0};
    return sendFrame(0, data);  // NMT消息的COB-ID是0
}


void CANopenDriver::registerPDOCallback(uint8_t pdo_num, std::function<void(const std::vector<uint8_t>&)> callback) {
    if (pdo_num < 1 || pdo_num > 4) {
        return;
    }

    uint32_t cob_id = calculateTPDOCobId(pdo_num);

    std::lock_guard<std::mutex> lock(callback_mutex_);
    pdo_callbacks_[cob_id] = callback;
}

bool CANopenDriver::sendSync() {
    std::vector<uint8_t> data = {0, 0, 0, 0, 0, 0, 0, 0};
    return sendFrame(0x80, data);  // SYNC消息的COB-ID是0x80
}

void CANopenDriver::receiveThread() {
    struct can_frame frame;

    while (running_) {
        ssize_t nbytes = read(can_socket_, &frame, sizeof(struct can_frame));

        if (nbytes < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                std::cerr << "Error reading from socket: " << strerror(errno) << std::endl;
            }
            continue;
        }

        if (nbytes < static_cast<ssize_t>(sizeof(struct can_frame))) {
            std::cerr << "Incomplete CAN frame received" << std::endl;
            continue;
        }

        // 处理接收到的帧
        uint32_t can_id = frame.can_id & CAN_EFF_MASK;  // 去掉扩展帧标志

        // 检查是否有注册的回调函数
        std::lock_guard<std::mutex> lock(callback_mutex_);
        auto it = pdo_callbacks_.find(can_id);
        if (it != pdo_callbacks_.end()) {
            // 提取数据
            std::vector<uint8_t> data(frame.data, frame.data + frame.can_dlc);

            // 调用回调函数
            it->second(data);
        }
    }
}

bool CANopenDriver::sendFrame(uint32_t can_id, const std::vector<uint8_t>& data) {
    if (can_socket_ < 0) {
        std::cerr << "CAN socket not initialized" << std::endl;
        return false;
    }

    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));

    frame.can_id = can_id;
    frame.can_dlc = data.size() > 8 ? 8 : data.size();

    for (size_t i = 0; i < frame.can_dlc; ++i) {
        frame.data[i] = data[i];
    }

    ssize_t nbytes = write(can_socket_, &frame, sizeof(struct can_frame));
    if (nbytes != sizeof(struct can_frame)) {
        std::cerr << "Error writing to socket: " << strerror(errno) << std::endl;
        return false;
    }

    return true;
}

bool CANopenDriver::sendPDO(uint8_t pdo_num, const std::vector<uint8_t>& data) {
    if (pdo_num < 1 || pdo_num > 4 || data.size() > 8) {
        std::cerr << "Invalid PDO parameters" << std::endl;
        return false;
    }

    uint32_t cob_id = calculateRPDOCobId(pdo_num);
    std::cout << "Sending PDO: COB-ID 0x" << std::hex << cob_id
              << ", Data: ";
    for (auto byte : data) {
        std::cout << std::hex << static_cast<int>(byte) << " ";
    }
    std::cout << std::dec << std::endl;

    return sendFrame(cob_id, data);
}

uint32_t CANopenDriver::calculateRPDOCobId(uint8_t pdo_num) const {
    // RPDO COB-ID: 0x200 + NodeID (RPDO1), 0x300 + NodeID (RPDO2), 0x400 + NodeID (RPDO3), 0x500 + NodeID (RPDO4)
    return (0x200 + (pdo_num - 1) * 0x100) + node_id_;
}

uint32_t CANopenDriver::calculateTPDOCobId(uint8_t pdo_num) const {
    // TPDO COB-ID: 0x180 + NodeID (TPDO1), 0x280 + NodeID (TPDO2), 0x380 + NodeID (TPDO3), 0x480 + NodeID (TPDO4)
    return (0x180 + (pdo_num - 1) * 0x100) + node_id_;
}

// 特化模板函数实现
template<>
bool CANopenDriver::readSDO<uint8_t>(uint16_t index, uint8_t subindex, uint8_t& value, int timeout_ms, int retries) {
    // SDO读取请求 (0x40 = 读取命令)
    std::vector<uint8_t> request = {0x40,
                                   static_cast<uint8_t>(index & 0xFF),
                                   static_cast<uint8_t>((index >> 8) & 0xFF),
                                   subindex, 0, 0, 0, 0};

    // 发送SDO请求
    uint32_t cob_id_tx = 0x600 + node_id_;  // SDO客户端到服务器
    uint32_t cob_id_rx = 0x580 + node_id_;  // SDO服务器到客户端

    // 清空接收缓冲区
    struct can_frame frame;
    while (recv(can_socket_, &frame, sizeof(frame), MSG_DONTWAIT) > 0) {
        // 丢弃所有待处理的帧
    }

    // 重试循环
    for (int attempt = 0; attempt < retries; ++attempt) {
        // 发送请求
        if (!sendFrame(cob_id_tx, request)) {
            std::cerr << "Failed to send SDO read request, attempt " << (attempt + 1) << std::endl;
            continue;
        }

        // 等待响应
        struct timeval tv;
        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;

        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(can_socket_, &readfds);

        int select_result = select(can_socket_ + 1, &readfds, NULL, NULL, &tv);

        if (select_result < 0) {
            std::cerr << "Select error: " << strerror(errno) << std::endl;
            continue;
        } else if (select_result == 0) {
            std::cerr << "SDO read timeout, attempt " << (attempt + 1) << std::endl;
            continue;
        }

        // 读取响应
        ssize_t nbytes = read(can_socket_, &frame, sizeof(frame));
        if (nbytes < static_cast<ssize_t>(sizeof(struct can_frame))) {
            std::cerr << "Incomplete CAN frame received" << std::endl;
            continue;
        }

        // 检查是否是我们期望的响应
        if ((frame.can_id & CAN_EFF_MASK) != cob_id_rx) {
            // 不是我们期望的响应，继续等待
            attempt--;  // 不计入重试次数
            continue;
        }

        // 检查命令字节 (0x4F = 1字节数据的成功响应)
        if (frame.data[0] == 0x4F) {
            // 检查索引和子索引是否匹配
            if (frame.data[1] == (index & 0xFF) &&
                frame.data[2] == ((index >> 8) & 0xFF) &&
                frame.data[3] == subindex) {

                // 解析数据
                value = frame.data[4];

                std::cout << "SDO read success: 0x" << std::hex << index << ":"
                          << static_cast<int>(subindex) << " = 0x"
                          << static_cast<int>(value) << std::dec << std::endl;

                return true;
            }
        } else if (frame.data[0] == 0x80) {
            // 错误响应
            uint32_t error_code = static_cast<uint32_t>(frame.data[4]) |
                                 (static_cast<uint32_t>(frame.data[5]) << 8) |
                                 (static_cast<uint32_t>(frame.data[6]) << 16) |
                                 (static_cast<uint32_t>(frame.data[7]) << 24);

            std::cerr << "SDO read error: 0x" << std::hex << error_code << std::dec << std::endl;
            return false;
        }
    }

    std::cerr << "SDO read failed after " << retries << " attempts" << std::endl;
    return false;
}

template<>
bool CANopenDriver::readSDO<uint16_t>(uint16_t index, uint8_t subindex, uint16_t& value, int timeout_ms, int retries) {
    // SDO读取请求 (0x40 = 读取命令)
    std::vector<uint8_t> request = {0x40,
                                   static_cast<uint8_t>(index & 0xFF),
                                   static_cast<uint8_t>((index >> 8) & 0xFF),
                                   subindex, 0, 0, 0, 0};

    // 发送SDO请求
    uint32_t cob_id_tx = 0x600 + node_id_;  // SDO客户端到服务器
    uint32_t cob_id_rx = 0x580 + node_id_;  // SDO服务器到客户端

    // 清空接收缓冲区
    struct can_frame frame;
    while (recv(can_socket_, &frame, sizeof(frame), MSG_DONTWAIT) > 0) {
        // 丢弃所有待处理的帧
    }

    // 重试循环
    for (int attempt = 0; attempt < retries; ++attempt) {
        // 发送请求
        if (!sendFrame(cob_id_tx, request)) {
            std::cerr << "Failed to send SDO read request, attempt " << (attempt + 1) << std::endl;
            continue;
        }

        // 等待响应
        struct timeval tv;
        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;

        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(can_socket_, &readfds);

        int select_result = select(can_socket_ + 1, &readfds, NULL, NULL, &tv);

        if (select_result < 0) {
            std::cerr << "Select error: " << strerror(errno) << std::endl;
            continue;
        } else if (select_result == 0) {
            std::cerr << "SDO read timeout, attempt " << (attempt + 1) << std::endl;
            continue;
        }

        // 读取响应
        ssize_t nbytes = read(can_socket_, &frame, sizeof(frame));
        if (nbytes < static_cast<ssize_t>(sizeof(struct can_frame))) {
            std::cerr << "Incomplete CAN frame received" << std::endl;
            continue;
        }

        // 检查是否是我们期望的响应
        if ((frame.can_id & CAN_EFF_MASK) != cob_id_rx) {
            // 不是我们期望的响应，继续等待
            attempt--;  // 不计入重试次数
            continue;
        }

        // 检查命令字节 (0x4B = 2字节数据的成功响应)
        if (frame.data[0] == 0x4B || frame.data[0] == 0x4F) {
            // 检查索引和子索引是否匹配
            if (frame.data[1] == (index & 0xFF) &&
                frame.data[2] == ((index >> 8) & 0xFF) &&
                frame.data[3] == subindex) {

                // 解析数据 (小端序)
                value = static_cast<uint16_t>(frame.data[4]) |
                        (static_cast<uint16_t>(frame.data[5]) << 8);

                std::cout << "SDO read success: 0x" << std::hex << index << ":"
                          << static_cast<int>(subindex) << " = 0x"
                          << value << std::dec << std::endl;

                return true;
            }
        } else if (frame.data[0] == 0x80) {
            // 错误响应
            uint32_t error_code = static_cast<uint32_t>(frame.data[4]) |
                                 (static_cast<uint32_t>(frame.data[5]) << 8) |
                                 (static_cast<uint32_t>(frame.data[6]) << 16) |
                                 (static_cast<uint32_t>(frame.data[7]) << 24);

            std::cerr << "SDO read error: 0x" << std::hex << error_code << std::dec << std::endl;
            return false;
        }
    }

    std::cerr << "SDO read failed after " << retries << " attempts" << std::endl;
    return false;
}

template<>
bool CANopenDriver::readSDO<uint32_t>(uint16_t index, uint8_t subindex, uint32_t& value, int timeout_ms, int retries) {
    // SDO读取请求 (0x40 = 读取命令)
    std::vector<uint8_t> request = {0x40,
                                   static_cast<uint8_t>(index & 0xFF),
                                   static_cast<uint8_t>((index >> 8) & 0xFF),
                                   subindex, 0, 0, 0, 0};

    // 发送SDO请求
    uint32_t cob_id_tx = 0x600 + node_id_;  // SDO客户端到服务器
    uint32_t cob_id_rx = 0x580 + node_id_;  // SDO服务器到客户端

    // 清空接收缓冲区
    struct can_frame frame;
    while (recv(can_socket_, &frame, sizeof(frame), MSG_DONTWAIT) > 0) {
        // 丢弃所有待处理的帧
    }

    // 重试循环
    for (int attempt = 0; attempt < retries; ++attempt) {
        // 发送请求
        if (!sendFrame(cob_id_tx, request)) {
            std::cerr << "Failed to send SDO read request, attempt " << (attempt + 1) << std::endl;
            continue;
        }

        // 等待响应
        struct timeval tv;
        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;

        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(can_socket_, &readfds);

        int select_result = select(can_socket_ + 1, &readfds, NULL, NULL, &tv);

        if (select_result < 0) {
            std::cerr << "Select error: " << strerror(errno) << std::endl;
            continue;
        } else if (select_result == 0) {
            std::cerr << "SDO read timeout, attempt " << (attempt + 1) << std::endl;
            continue;
        }

        // 读取响应
        ssize_t nbytes = read(can_socket_, &frame, sizeof(frame));
        if (nbytes < static_cast<ssize_t>(sizeof(struct can_frame))) {
            std::cerr << "Incomplete CAN frame received" << std::endl;
            continue;
        }

        // 检查是否是我们期望的响应
        if ((frame.can_id & CAN_EFF_MASK) != cob_id_rx) {
            // 不是我们期望的响应，继续等待
            attempt--;  // 不计入重试次数
            continue;
        }

        // 检查命令字节 (0x43 = 4字节数据的成功响应)
        if (frame.data[0] == 0x43 || frame.data[0] == 0x4F) {
            // 检查索引和子索引是否匹配
            if (frame.data[1] == (index & 0xFF) &&
                frame.data[2] == ((index >> 8) & 0xFF) &&
                frame.data[3] == subindex) {

                // 解析数据 (小端序)
                value = static_cast<uint32_t>(frame.data[4]) |
                       (static_cast<uint32_t>(frame.data[5]) << 8) |
                       (static_cast<uint32_t>(frame.data[6]) << 16) |
                       (static_cast<uint32_t>(frame.data[7]) << 24);

                std::cout << "SDO read success: 0x" << std::hex << index << ":"
                          << static_cast<int>(subindex) << " = 0x"
                          << value << std::dec << std::endl;

                return true;
            }
        } else if (frame.data[0] == 0x80) {
            // 错误响应
            uint32_t error_code = static_cast<uint32_t>(frame.data[4]) |
                                 (static_cast<uint32_t>(frame.data[5]) << 8) |
                                 (static_cast<uint32_t>(frame.data[6]) << 16) |
                                 (static_cast<uint32_t>(frame.data[7]) << 24);

            std::cerr << "SDO read error: 0x" << std::hex << error_code << std::dec << std::endl;
            return false;
        }
    }

    std::cerr << "SDO read failed after " << retries << " attempts" << std::endl;
    return false;
}

template<>
bool CANopenDriver::writeSDO<uint8_t>(uint16_t index, uint8_t subindex, const uint8_t& value) {
    std::vector<uint8_t> request = {0x2F, static_cast<uint8_t>(index & 0xFF), static_cast<uint8_t>((index >> 8) & 0xFF), subindex, value, 0, 0, 0};
    uint32_t cob_id = 0x600 + node_id_;
    return sendFrame(cob_id, request);
}

template<>
bool CANopenDriver::writeSDO<uint16_t>(uint16_t index, uint8_t subindex, const uint16_t& value) {
    std::vector<uint8_t> request = {0x2B, static_cast<uint8_t>(index & 0xFF), static_cast<uint8_t>((index >> 8) & 0xFF), subindex,
                                   static_cast<uint8_t>(value & 0xFF), static_cast<uint8_t>((value >> 8) & 0xFF), 0, 0};
    uint32_t cob_id = 0x600 + node_id_;
    return sendFrame(cob_id, request);
}

template<>
bool CANopenDriver::writeSDO<uint32_t>(uint16_t index, uint8_t subindex, const uint32_t& value) {
    std::vector<uint8_t> request = {0x23, static_cast<uint8_t>(index & 0xFF), static_cast<uint8_t>((index >> 8) & 0xFF), subindex,
                                   static_cast<uint8_t>(value & 0xFF), static_cast<uint8_t>((value >> 8) & 0xFF),
                                   static_cast<uint8_t>((value >> 16) & 0xFF), static_cast<uint8_t>((value >> 24) & 0xFF)};
    uint32_t cob_id = 0x600 + node_id_;
    return sendFrame(cob_id, request);
}

template<>
bool CANopenDriver::readSDO<int32_t>(uint16_t index, uint8_t subindex, int32_t& value, int timeout_ms, int retries) {
    uint32_t temp;
    bool result = readSDO<uint32_t>(index, subindex, temp, timeout_ms, retries);
    if (result) {
        value = static_cast<int32_t>(temp);
    }
    return result;
}

// 通用模板实现，用于处理未特化的类型
template<typename T>
bool CANopenDriver::readSDO(uint16_t index, uint8_t subindex, T& value, int timeout_ms, int retries) {
    std::cerr << "Generic readSDO not implemented for this type" << std::endl;
    return false;
}

} // namespace yz_motor_driver
