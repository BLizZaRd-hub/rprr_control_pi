#ifndef SYNC_MASTER_HPP
#define SYNC_MASTER_HPP

#include <atomic>
#include <thread>
#include <chrono>
#include <string>
#include <memory>
#include <functional>

// 前向声明，避免包含完整的ROS头文件
namespace rclcpp {
class Logger;
}

namespace yz_motor_driver {

class SyncMaster {
public:
    SyncMaster(const std::string& interface = "can0", 
               uint32_t period_ns = 1000000,  // 1kHz
               double alpha = 0.05);
    ~SyncMaster();

    // 启动SYNC主时钟
    bool start();
    
    // 停止SYNC主时钟
    void stop();
    
    // 设置SYNC周期
    void setPeriod(uint32_t period_ns);
    
    // 设置IIR滤波系数
    void setAlpha(double alpha);
    
    // 获取当前状态
    bool isRunning() const { return running_; }
    int64_t getPhaseError() const { return phase_err_; }
    uint32_t getDropCount() const { return drop_counter_; }
    double getBusLoad() const { return bus_load_; }

private:
    // CAN接口名称
    std::string interface_;
    
    // 时钟参数
    uint32_t period_ns_;
    double alpha_;
    int64_t phase_err_;
    uint64_t deadline_ns_;
    uint32_t drop_counter_;
    double bus_load_;
    
    // 运行状态
    std::atomic<bool> running_;
    std::thread sync_thread_;
    int can_socket_;
    
    // 主时钟线程函数
    void syncThreadFunc();
    
    // 初始化CAN socket
    bool initSocket();
    
    // 关闭CAN socket
    void closeSocket();
    
    // 设置线程实时属性
    bool setRealtimeAttributes();
    
    // 发送SYNC帧并获取硬件时间戳
    bool sendSyncAndGetTimestamp(uint64_t& tx_timestamp);
    
    // 计算总线负载
    void updateBusLoad();
};

} // namespace yz_motor_driver

#endif // SYNC_MASTER_HPP
