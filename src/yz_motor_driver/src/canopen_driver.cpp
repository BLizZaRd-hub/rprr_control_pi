#include "yz_motor_driver/canopen_driver.hpp"
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <string.h>
#include <iostream>

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
bool CANopenDriver::readSDO<uint8_t>(uint16_t index, uint8_t subindex, uint8_t& value) {
    // 实际实现中需要发送SDO请求并等待响应
    // 这里简化处理，实际应用需要完善
    return true;
}

template<>
bool CANopenDriver::readSDO<uint16_t>(uint16_t index, uint8_t subindex, uint16_t& value) {
    // 实际实现中需要发送SDO请求并等待响应
    // 这里简化处理，实际应用需要完善
    return true;
}

template<>
bool CANopenDriver::readSDO<uint32_t>(uint16_t index, uint8_t subindex, uint32_t& value) {
    // 实际实现中需要发送SDO请求并等待响应
    // 这里简化处理，实际应用需要完善
    return true;
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

} // namespace yz_motor_driver
