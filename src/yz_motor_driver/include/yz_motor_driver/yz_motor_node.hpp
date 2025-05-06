#ifndef YZ_MOTOR_NODE_HPP
#define YZ_MOTOR_NODE_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>  // 添加 Bool 消息类型
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>

// 添加服务接口头文件
#include "yz_motor_driver/srv/set_position_reached_params.hpp"

#include "yz_motor_driver/canopen_driver.hpp"
#include "yz_motor_driver/cia402_driver.hpp"

namespace yz_motor_driver {

class YZMotorNode : public rclcpp::Node {
public:
    YZMotorNode();
    ~YZMotorNode();

private:
    // 初始化电机
    bool initMotor();
    
    // 转换RPM到内部速度单位
    int32_t rpmToVelocity(float rpm);
    
    // 回调函数
    void positionCmdCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void positionDegCmdCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void velocityCmdCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void velocityRpmCmdCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void statusTimerCallback();
    
    // 服务回调
    void enableCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void disableCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void setPositionModeCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                               std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    
    // 驱动实例
    std::shared_ptr<CANopenDriver> canopen_driver_;
    std::shared_ptr<CiA402Driver> cia402_driver_;
    
    // 发布者
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr position_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr position_deg_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity_rpm_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr position_reached_pub_;
    
    // 订阅者
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr position_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr position_deg_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr velocity_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr velocity_rpm_cmd_sub_;
    
    // 服务
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr position_mode_srv_;
    rclcpp::Service<yz_motor_driver::srv::SetPositionReachedParams>::SharedPtr set_position_reached_params_srv_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr status_timer_;
    
    // 状态变量
    bool last_target_reached_ = false;
    
    // 到位检测参数服务回调
    void setPositionReachedParamsCallback(
        const std::shared_ptr<yz_motor_driver::srv::SetPositionReachedParams::Request> request,
        std::shared_ptr<yz_motor_driver::srv::SetPositionReachedParams::Response> response);
};

} // namespace yz_motor_driver

#endif // YZ_MOTOR_NODE_HPP
