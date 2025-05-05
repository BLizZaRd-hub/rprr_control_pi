#ifndef SYNC_MASTER_NODE_HPP
#define SYNC_MASTER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "yz_motor_driver/sync_master.hpp"

namespace yz_motor_driver {

class SyncMasterNode : public rclcpp::Node {
public:
    SyncMasterNode();
    ~SyncMasterNode();

private:
    // 参数
    std::string can_interface_;
    uint32_t sync_period_ns_;
    double iir_alpha_;
    
    // SYNC主时钟实例
    std::unique_ptr<SyncMaster> sync_master_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;
    
    // 发布者
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr diagnostics_pub_;
    
    // 服务
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_period_srv_;
    
    // 回调函数
    void diagnosticsTimerCallback();
    void startCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void stopCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void setPeriodCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                          std::shared_ptr<std_srvs::srv::SetBool::Response> response);
};

} // namespace yz_motor_driver

#endif // SYNC_MASTER_NODE_HPP