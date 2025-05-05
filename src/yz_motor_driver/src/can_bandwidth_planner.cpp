#include "yz_motor_driver/can_bandwidth_planner.hpp"
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>

namespace yz_motor_driver {

CANBandwidthPlanner::CANBandwidthPlanner(uint32_t bitrate, double safe_util)
    : bitrate_(bitrate), safe_util_(safe_util) {
}

void CANBandwidthPlanner::addFrame(const BusFrame& frame) {
    frames_.push_back(frame);
}

void CANBandwidthPlanner::addFrame(uint32_t cob_id, uint8_t dlc, double freq_hz, const std::string& description) {
    frames_.push_back(BusFrame(cob_id, dlc, freq_hz, description));
}

void CANBandwidthPlanner::addSYNC(double freq_hz) {
    addFrame(0x080, 0, freq_hz, "master SYNC");
}

void CANBandwidthPlanner::addRPDO1(uint8_t node_id, uint8_t dlc, double freq_hz) {
    addFrame(0x200 + node_id, dlc, freq_hz, "node" + std::to_string(node_id) + "_RPDO1");
}

void CANBandwidthPlanner::addTPDO1(uint8_t node_id, uint8_t dlc, double freq_hz) {
    addFrame(0x180 + node_id, dlc, freq_hz, "node" + std::to_string(node_id) + "_TPDO1");
}

void CANBandwidthPlanner::addHeartbeat(uint8_t node_id, double freq_hz) {
    addFrame(0x700 + node_id, 1, freq_hz, "node" + std::to_string(node_id) + "_heartbeat");
}

void CANBandwidthPlanner::addMultiAxisFrames(uint8_t start_node, uint8_t num_nodes, 
                                           uint8_t rpdo_dlc, uint8_t tpdo_dlc, 
                                           double pdo_freq_hz, double heartbeat_freq_hz) {
    for (uint8_t i = 0; i < num_nodes; ++i) {
        uint8_t node_id = start_node + i;
        addRPDO1(node_id, rpdo_dlc, pdo_freq_hz);
        addTPDO1(node_id, tpdo_dlc, pdo_freq_hz);
        addHeartbeat(node_id, heartbeat_freq_hz);
    }
}

uint32_t CANBandwidthPlanner::frameBits(const BusFrame& frame) const {
    // 确保DLC在有效范围内
    uint8_t dlc = std::min(frame.dlc, MAX_DLC);
    
    // 计算数据位数
    uint32_t payload_bits = dlc * 8;
    
    // 返回总位数
    return OVERHEAD_BITS + payload_bits;
}

double CANBandwidthPlanner::frameBandwidth(const BusFrame& frame) const {
    return frameBits(frame) * frame.freq;  // bits per second
}

double CANBandwidthPlanner::calculateUtilization() const {
    double total_bits = 0.0;
    
    for (const auto& frame : frames_) {
        total_bits += frameBandwidth(frame);
    }
    
    return total_bits / bitrate_;  // 占用率 (0-1)
}

void CANBandwidthPlanner::adjustSyncDependentFrames(double new_sync_freq) {
    // 找到SYNC帧并更新频率
    for (auto& frame : frames_) {
        if (frame.cob_id == 0x080 && frame.desc.find("SYNC") != std::string::npos) {
            frame.freq = new_sync_freq;
            break;
        }
    }
    
    // 更新所有依赖SYNC的PDO帧频率
    // 假设所有RPDO1和TPDO1都是SYNC触发的
    for (auto& frame : frames_) {
        if ((frame.cob_id >= 0x180 && frame.cob_id < 0x200) ||  // TPDO1
            (frame.cob_id >= 0x200 && frame.cob_id < 0x280)) {  // RPDO1
            frame.freq = new_sync_freq;
        }
    }
}

bool CANBandwidthPlanner::autoAdjust() {
    double utilization = calculateUtilization();
    
    // 如果已经在安全范围内，不需要调整
    if (utilization <= safe_util_) {
        return true;
    }
    
    // 找到当前SYNC频率
    double sync_freq = 1000.0;
    for (const auto& frame : frames_) {
        if (frame.cob_id == 0x080 && frame.desc.find("SYNC") != std::string::npos) {
            sync_freq = frame.freq;
            break;
        }
    }
    
    // 尝试降低SYNC频率
    while (utilization > safe_util_ && sync_freq > 250.0) {
        sync_freq /= 2.0;
        adjustSyncDependentFrames(sync_freq);
        utilization = calculateUtilization();
    }
    
    // 如果调整后仍然超过安全阈值，返回失败
    return utilization <= safe_util_;
}

std::string CANBandwidthPlanner::getSummary() const {
    std::ostringstream oss;
    double utilization = calculateUtilization();
    
    oss << "CAN总线带宽规划摘要:\n";
    oss << "-------------------\n";
    oss << "总线比特率: " << bitrate_ / 1000000.0 << " Mbit/s\n";
    oss << "安全阈值: " << safe_util_ * 100.0 << "%\n";
    oss << "当前占用率: " << std::fixed << std::setprecision(1) << utilization * 100.0 << "%\n";
    oss << "状态: " << (utilization <= safe_util_ ? "正常" : "超载") << "\n\n";
    
    oss << "帧详情:\n";
    oss << "COB-ID  DLC  频率(Hz)  带宽(bit/s)  描述\n";
    oss << "------  ---  --------  -----------  ----\n";
    
    for (const auto& frame : frames_) {
        oss << "0x" << std::hex << std::setw(3) << std::setfill('0') << frame.cob_id << "   ";
        oss << std::dec << static_cast<int>(frame.dlc) << "    ";
        oss << std::setw(8) << std::fixed << std::setprecision(1) << frame.freq << "  ";
        oss << std::setw(11) << std::fixed << std::setprecision(1) << frameBandwidth(frame) << "  ";
        oss << frame.desc << "\n";
    }
    
    return oss.str();
}

void CANBandwidthPlanner::clear() {
    frames_.clear();
}

} // namespace yz_motor_driver