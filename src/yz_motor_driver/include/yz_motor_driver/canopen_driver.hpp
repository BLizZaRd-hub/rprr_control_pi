#ifndef CANOPEN_DRIVER_HPP
#define CANOPEN_DRIVER_HPP

#include <string>
#include <vector>
#include <functional>
#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <unordered_map>

namespace yz_motor_driver {

// CANopen通信类
class CANopenDriver {
public:
    CANopenDriver(const std::string& interface, uint8_t node_id);
    ~CANopenDriver();

    // 初始化CAN接口
    bool init();

    // NMT功能
    bool resetNode();
    bool setPreOperational();
    bool startRemoteNode();
    bool stopRemoteNode();

    // SDO通信（读写对象字典）
    template<typename T>
    bool readSDO(uint16_t index, uint8_t subindex, T& value);

    template<typename T>
    bool writeSDO(uint16_t index, uint8_t subindex, const T& value);

    // PDO通信
    bool sendPDO(uint8_t pdo_num, const std::vector<uint8_t>& data);

    // 注册PDO接收回调
    void registerPDOCallback(uint8_t pdo_num, std::function<void(const std::vector<uint8_t>&)> callback);

    // 发送SYNC消息
    bool sendSync();

    // 获取节点ID
    uint8_t getNodeId() const { return node_id_; }

private:
    int can_socket_ = -1;
    std::string interface_;
    uint8_t node_id_;

    std::thread receive_thread_;
    std::atomic<bool> running_{false};

    std::unordered_map<uint32_t, std::function<void(const std::vector<uint8_t>&)>> pdo_callbacks_;
    std::mutex callback_mutex_;

    // 接收线程函数
    void receiveThread();

    // 发送CAN帧
    bool sendFrame(uint32_t can_id, const std::vector<uint8_t>& data);

    // 计算PDO的COB-ID
    uint32_t calculateRPDOCobId(uint8_t pdo_num) const;
    uint32_t calculateTPDOCobId(uint8_t pdo_num) const;
};

// 模板函数实现
template<typename T>
bool CANopenDriver::readSDO(uint16_t index, uint8_t subindex, T& value) {
    // 模拟读取SDO
    // 在实际实现中，这里应该发送SDO读取请求并等待响应
    
    // 为了避免未使用参数的警告
    (void)index;
    (void)subindex;
    (void)value;
    
    // 返回成功
    return true;
}

template<typename T>
bool CANopenDriver::writeSDO(uint16_t index, uint8_t subindex, const T& value) {
    std::vector<uint8_t> request = {0x23, static_cast<uint8_t>(index & 0xFF), static_cast<uint8_t>((index >> 8) & 0xFF), subindex};

    // 添加数据（小端序）
    for (size_t i = 0; i < sizeof(T); ++i) {
        request.push_back(static_cast<uint8_t>((value >> (i * 8)) & 0xFF));
    }

    // 填充到8字节
    while (request.size() < 8) {
        request.push_back(0);
    }

    uint32_t cob_id = 0x600 + node_id_;
    return sendFrame(cob_id, request);
}

} // namespace yz_motor_driver

#endif // CANOPEN_DRIVER_HPP
