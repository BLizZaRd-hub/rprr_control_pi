#ifndef YZ_MOTOR_NODE_HPP
#define YZ_MOTOR_NODE_HPP

#include "yz_motor_driver/cia402_driver.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
// 移除不存在的头文件
// #include <example_interfaces/srv/set_int64.hpp>
// 使用标准的整数服务
#include <std_srvs/srv/empty.hpp>
#include <memory>

namespace yz_motor_driver {

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
    int current_profile_velocity_;
    int current_profile_acceleration_;
    
    // ROS2服务
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr position_mode_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr velocity_mode_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_params_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_velocity_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_acceleration_srv_;
    
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
    void setVelocityCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    void setAccelerationCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    
    void positionCmdCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void positionDegCmdCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void velocityCmdCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void velocityRpmCmdCallback(const std_msgs::msg::Float32::SharedPtr msg);
    
    void statusTimerCallback();
    
    // 辅助函数
    int32_t degreesToEncoder(double degrees);
    double encoderToDegrees(int32_t encoder);
    int32_t rpmToVelocity(double rpm);
    double velocityToRpm(int32_t velocity);
};

} // namespace yz_motor_driver

#endif // YZ_MOTOR_NODE_HPP
