#ifndef YZ_MOTOR_NODE_HPP
#define YZ_MOTOR_NODE_HPP

#include "yz_motor_driver/cia402_driver.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <memory>

namespace yz_motor_driver {

// 定义我们自己的服务消息类型
namespace srv {
  struct SetInt64 {
    struct Request {
      int64_t data;
    };
    struct Response {
      bool success;
    };
  };
}

class YZMotorNode : public rclcpp::Node {
public:
    YZMotorNode();
    ~YZMotorNode();
    
private:
    // 驱动实例
    std::shared_ptr<CANopenDriver> canopen_driver_;
    std::shared_ptr<CiA402Driver> cia402_driver_;
    
    // ROS2参数
    std::string can_interface_;
    int node_id_;
    double position_scale_;
    double velocity_scale_;
    
    // ROS2服务
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr position_mode_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr velocity_mode_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_params_srv_;
    
    // 使用std_srvs::srv::Trigger替代SetInt64
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_velocity_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_acceleration_srv_;
    
    // 存储当前值
    uint32_t current_profile_velocity_;
    uint32_t current_profile_acceleration_;
    
    // ROS2话题
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr position_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr position_deg_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity_rpm_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr status_pub_;
    
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr position_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr position_deg_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr velocity_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr velocity_rpm_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr position_deg_relative_cmd_sub_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr status_timer_;
    
    // 回调函数
    void enableCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void disableCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void homeCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void positionModeCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                             std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    void velocityModeCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                             std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    void saveParamsCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    // 修改为使用Trigger
    void setVelocityCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void setAccelerationCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    void positionCmdCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void positionDegCmdCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void positionDegRelativeCmdCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void velocityCmdCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void velocityRpmCmdCallback(const std_msgs::msg::Float32::SharedPtr msg);
    
    void statusTimerCallback();
    
    // 辅助函数
    double encoderToDegrees(int32_t encoder_value);
    int32_t degreesToEncoder(double degrees);
    double velocityToRpm(int32_t velocity);
    int32_t rpmToVelocity(double rpm);
};

} // namespace yz_motor_driver

#endif // YZ_MOTOR_NODE_HPP
