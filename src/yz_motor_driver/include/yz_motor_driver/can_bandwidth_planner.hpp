#ifndef CAN_BANDWIDTH_PLANNER_HPP
#define CAN_BANDWIDTH_PLANNER_HPP

#include <vector>
#include <string>
#include <stdexcept>
#include <cstdint>
#include <functional>

namespace yz_motor_driver {

// 总线带宽规划异常
class PlanningError : public std::runtime_error {
public:
    explicit PlanningError(const std::string& msg) : std::runtime_error(msg) {}
};

// 通信实体类
class BusFrame {
public:
    BusFrame(uint32_t cob_id, uint8_t dlc, double freq_hz, const std::string& description)
        : cob_id(cob_id), dlc(dlc), freq(freq_hz), desc(description) {}

    uint32_t cob_id;      // 决定优先级
    uint8_t dlc;          // 数据长度 0-8
    double freq;          // 触发频率 (Hz)；0表示事件触发
    std::string desc;     // 备注
};

// 带宽规划器类
class CANBandwidthPlanner {
public:
    CANBandwidthPlanner(uint32_t bitrate = 1000000, double safe_util = 0.30);

    // 添加帧
    void addFrame(const BusFrame& frame);
    void addFrame(uint32_t cob_id, uint8_t dlc, double freq_hz, const std::string& description);

    // 添加常见帧类型
    void addSYNC(double freq_hz = 1000.0);
    void addRPDO1(uint8_t node_id, uint8_t dlc, double freq_hz);
    void addTPDO1(uint8_t node_id, uint8_t dlc, double freq_hz);
    void addHeartbeat(uint8_t node_id, double freq_hz = 5.0);

    // 批量添加多轴
    void addMultiAxisFrames(uint8_t start_node, uint8_t num_nodes, 
                           uint8_t rpdo_dlc, uint8_t tpdo_dlc, 
                           double pdo_freq_hz, double heartbeat_freq_hz = 5.0);

    // 计算带宽
    double calculateUtilization() const;
    
    // 自动调参
    bool autoAdjust();
    
    // 获取结果
    std::string getSummary() const;
    
    // 清除所有帧
    void clear();
    
    // 获取安全利用率
    double getSafeUtilization() const { return safe_util_; }

private:
    static constexpr uint32_t OVERHEAD_BITS = 47;  // 11位ID帧：SOF+ID+RTR+IDE+...+CRC+EOF+IFS
    static constexpr uint8_t MAX_DLC = 8;
    
    uint32_t bitrate_;           // 总线比特率
    double safe_util_;           // 安全利用率
    std::vector<BusFrame> frames_;  // 所有帧
    
    // 计算单帧位数
    uint32_t frameBits(const BusFrame& frame) const;
    
    // 计算单帧带宽
    double frameBandwidth(const BusFrame& frame) const;
    
    // 调整SYNC相关帧频率
    void adjustSyncDependentFrames(double new_sync_freq);
};

} // namespace yz_motor_driver

#endif // CAN_BANDWIDTH_PLANNER_HPP
