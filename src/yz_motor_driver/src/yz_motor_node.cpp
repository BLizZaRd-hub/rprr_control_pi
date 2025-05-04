#include "yz_motor_driver/yz_motor_node.hpp"
#include <iostream>
#include <cmath>

namespace yz_motor_driver {

YZMotorNode::YZMotorNode(const rclcpp::NodeOptions& options)
    : Node("yz_motor_node", options) {
    // 声明参数
    this->declare_parameter("can_interface", "can0");
    this->declare_parameter("node_id", 3);
    this->declare_parameter("position_scale", 10000.0);
    this->declare_parameter("velocity_scale", 1.0);
    this->declare_parameter("profile_velocity", 1000);
    this->declare_parameter("profile_acceleration", 1000);
    
    // 获取参数
    can_interface_ = this->get_parameter("can_interface").as_string();
    node_id_ = this->get_parameter("node_id").as_int();
    position_scale_ = this->get_parameter("position_scale").as_double();
    velocity_scale_ = this->get_parameter("velocity_scale").as_double();
    current_profile_velocity_ = this->get_parameter("profile_velocity").as_int();
    current_profile_acceleration_ = this->get_parameter("profile_acceleration").as_int();
    
    // 创建驱动实例
    canopen_driver_ = std::make_shared<CANopenDriver>(can_interface_, static_cast<uint8_t>(node_id_));
    if (!canopen_driver_->init()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize CANopen driver");
        return;
    }
    
    // 创建CiA402驱动
    cia402_driver_ = std::make_shared<CiA402Driver>(canopen_driver_);
    
    // 注意：不再自动尝试使能电机，而是等待使能服务被调用
    RCLCPP_INFO(this->get_logger(), "Motor driver initialized. Please call the enable service to enable the motor.");
    
    // 创建服务
    enable_service_ = this->create_service<std_srvs::srv::Trigger>(
        "enable", std::bind(&YZMotorNode::enableCallback, this, std::placeholders::_1, std::placeholders::_2));
    
    disable_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "disable",
        std::bind(&YZMotorNode::disableCallback, this, std::placeholders::_1, std::placeholders::_2));
    
    home_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "home",
        std::bind(&YZMotorNode::homeCallback, this, std::placeholders::_1, std::placeholders::_2));
    
    position_mode_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "position_mode",
        std::bind(&YZMotorNode::positionModeCallback, this, std::placeholders::_1, std::placeholders::_2));
    
    velocity_mode_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "velocity_mode",
        std::bind(&YZMotorNode::velocityModeCallback, this, std::placeholders::_1, std::placeholders::_2));
    
    save_params_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "save_params",
        std::bind(&YZMotorNode::saveParamsCallback, this, std::placeholders::_1, std::placeholders::_2));
    
    // 创建发布者
    position_pub_ = this->create_publisher<std_msgs::msg::Int32>("position", 10);
    position_deg_pub_ = this->create_publisher<std_msgs::msg::Float32>("position_deg", 10);
    velocity_pub_ = this->create_publisher<std_msgs::msg::Int32>("velocity", 10);
    velocity_rpm_pub_ = this->create_publisher<std_msgs::msg::Float32>("velocity_rpm", 10);
    status_pub_ = this->create_publisher<std_msgs::msg::UInt16>("status", 10);
    
    // 创建订阅者
    position_cmd_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "position_cmd", 10,
        std::bind(&YZMotorNode::positionCmdCallback, this, std::placeholders::_1));
    
    position_deg_cmd_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "position_deg_cmd", 10,
        std::bind(&YZMotorNode::positionDegCmdCallback, this, std::placeholders::_1));
    
    position_deg_relative_cmd_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "position_deg_relative_cmd", 10,
        std::bind(&YZMotorNode::positionDegRelativeCmdCallback, this, std::placeholders::_1));
    
    velocity_cmd_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "velocity_cmd", 10,
        std::bind(&YZMotorNode::velocityCmdCallback, this, std::placeholders::_1));
    
    velocity_rpm_cmd_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "velocity_rpm_cmd", 10,
        std::bind(&YZMotorNode::velocityRpmCmdCallback, this, std::placeholders::_1));
    
    // 只有在启用状态监控时才创建定时器
    if (enable_status_monitoring_) {
        status_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),  // 1秒更新一次
            std::bind(&YZMotorNode::updateStatus, this));
    }
    
    set_velocity_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "increase_velocity",
        std::bind(&YZMotorNode::setVelocityCallback, this, std::placeholders::_1, std::placeholders::_2));
    
    set_acceleration_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "increase_acceleration",
        std::bind(&YZMotorNode::setAccelerationCallback, this, std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "YZ Motor node initialized");
}

YZMotorNode::~YZMotorNode() {
    // 尝试禁用电机
    if (cia402_driver_) {
        cia402_driver_->disableOperation();
    }
}

void YZMotorNode::enableCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;  // 未使用
    
    if (!cia402_driver_) {
        response->success = false;
        response->message = "Driver not initialized";
        return;
    }
    
    // 获取当前状态
    CiA402State state = cia402_driver_->getState();
    RCLCPP_INFO(this->get_logger(), "Current motor state: %d", static_cast<int>(state));
    
    // 如果已经处于使能状态，直接返回成功
    if (state == CiA402State::OPERATION_ENABLED) {
        response->success = true;
        response->message = "Motor already enabled";
        return;
    }
    
    // 首先尝试使用SDO使能
    RCLCPP_INFO(this->get_logger(), "Enabling motor using SDO");
    if (cia402_driver_->enableOperation()) {
        response->success = true;
        response->message = "Motor enabled successfully via SDO";
        return;
    }
    
    RCLCPP_WARN(this->get_logger(), "Failed to enable motor using SDO");
    
    // 如果SDO使能失败，尝试使用PDO使能
    RCLCPP_INFO(this->get_logger(), "Trying to enable via PDO");
    if (cia402_driver_->enableOperationPDO()) {
        response->success = true;
        response->message = "Motor enabled successfully via PDO";
        return;
    }
    
    // 如果两种方式都失败，返回错误
    response->success = false;
    response->message = "Failed to enable motor via both SDO and PDO";
}

void YZMotorNode::disableCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;  // 未使用
    
    if (!cia402_driver_) {
        response->success = false;
        response->message = "Driver not initialized";
        return;
    }
    
    bool result = cia402_driver_->disableOperation();
    response->success = result;
    response->message = result ? "Motor disabled" : "Failed to disable motor";
}

void YZMotorNode::homeCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;  // 未使用
    
    if (!cia402_driver_) {
        response->success = false;
        response->message = "Driver not initialized";
        return;
    }
    
    // 设置回零模式
    if (!cia402_driver_->setOperationMode(OperationMode::HOMING)) {
        response->success = false;
        response->message = "Failed to set homing mode";
        return;
    }
    
    // 启动回零
    bool result = cia402_driver_->startHoming(17);  // 使用方法17
    response->success = result;
    response->message = result ? "Homing started" : "Failed to start homing";
}

void YZMotorNode::positionModeCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    if (!cia402_driver_) {
        response->success = false;
        response->message = "Driver not initialized";
        return;
    }
    
    if (request->data) {
        bool result = cia402_driver_->setOperationMode(OperationMode::PROFILE_POSITION);
        response->success = result;
        response->message = result ? "Position mode enabled" : "Failed to enable position mode";
    } else {
        response->success = true;
        response->message = "No action taken";
    }
}

void YZMotorNode::velocityModeCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    if (!cia402_driver_) {
        response->success = false;
        response->message = "Driver not initialized";
        return;
    }
    
    if (request->data) {
        bool result = cia402_driver_->setOperationMode(OperationMode::PROFILE_VELOCITY);
        response->success = result;
        response->message = result ? "Velocity mode enabled" : "Failed to enable velocity mode";
    } else {
        response->success = true;
        response->message = "No action taken";
    }
}

void YZMotorNode::saveParamsCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;  // 未使用
    
    if (!cia402_driver_) {
        response->success = false;
        response->message = "Driver not initialized";
        return;
    }
    
    bool result = cia402_driver_->saveParameters();
    response->success = result;
    response->message = result ? "Parameters saved" : "Failed to save parameters";
}

void YZMotorNode::positionCmdCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    if (!cia402_driver_) {
        return;
    }
    
    cia402_driver_->setTargetPositionPDO(msg->data);
}

void YZMotorNode::positionDegCmdCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received position_deg_cmd: %.2f", msg->data);
    
    if (!cia402_driver_) {
        RCLCPP_ERROR(this->get_logger(), "CiA402 driver not initialized");
        return;
    }
    
    int32_t position = degreesToEncoder(msg->data);
    RCLCPP_INFO(this->get_logger(), "Converted to encoder position: %d", position);
    
    // 修改为相对位置模式 (false)
    bool result = cia402_driver_->setTargetPositionPDO(position, false);
    RCLCPP_INFO(this->get_logger(), "setTargetPositionPDO result: %s", result ? "success" : "failed");
}

void YZMotorNode::positionDegRelativeCmdCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(), "Received position_deg_relative_cmd: %.2f", msg->data);
    
    if (!cia402_driver_) {
        RCLCPP_ERROR(this->get_logger(), "CiA402 driver not initialized");
        return;
    }
    
    // 1. 检查电机是否已经处于使能状态
    CiA402State state = cia402_driver_->getState();
    if (state != CiA402State::OPERATION_ENABLED) {
        RCLCPP_ERROR(this->get_logger(), "Motor not enabled. Current state: %d", static_cast<int>(state));
        RCLCPP_INFO(this->get_logger(), "Please call the enable service first");
        return;
    }
    
    // 2. 确保电机处于位置模式
    if (cia402_driver_->getOperationMode() != OperationMode::PROFILE_POSITION) {
        RCLCPP_INFO(this->get_logger(), "Setting operation mode to Profile Position");
        if (!cia402_driver_->setOperationMode(OperationMode::PROFILE_POSITION)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set Profile Position mode");
            return;
        }
        // 给驱动器更多时间来切换模式
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // 3. 将角度转换为编码器脉冲
    int32_t position = degreesToEncoder(msg->data);
    RCLCPP_DEBUG(this->get_logger(), "Converting %.2f degrees to %d encoder pulses (scale: %.2f)", 
                msg->data, position, position_scale_);
    
    // 4. 发送相对位置命令 (使用0x005F控制字，设置Bit6=1表示相对模式，Bit4=1触发运动)
    bool result = cia402_driver_->setRelativePositionCommand(position);
    RCLCPP_INFO(this->get_logger(), "Relative position command (%.2f deg) sent: %s", 
                msg->data, result ? "success" : "failed");
}

void YZMotorNode::velocityCmdCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    if (!cia402_driver_) {
        return;
    }
    
    cia402_driver_->setTargetVelocityPDO(msg->data);
}

void YZMotorNode::velocityRpmCmdCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    if (!cia402_driver_) {
        return;
    }
    
    int32_t velocity = rpmToVelocity(msg->data);
    cia402_driver_->setTargetVelocity(velocity);
}

void YZMotorNode::updateStatus() {
    if (!cia402_driver_ || !enable_status_monitoring_) {
        return;
    }
    
    // 更新状态
    CiA402State state = cia402_driver_->getState();
    int32_t position = cia402_driver_->getPosition();
    
    // 只在状态变化时打印日志
    static CiA402State last_state = CiA402State::NOT_READY_TO_SWITCH_ON;
    static int32_t last_position = 0;
    
    if (state != last_state) {
        RCLCPP_INFO(this->get_logger(), "Motor state changed: %d -> %d", 
                   static_cast<int>(last_state), static_cast<int>(state));
        last_state = state;
    }
    
    // 只有当位置变化超过阈值时才打印
    if (std::abs(position - last_position) > 100) {
        RCLCPP_DEBUG(this->get_logger(), "Motor position: %d", position);
        last_position = position;
    }
    
    // 发布位置
    auto position_msg = std::make_unique<std_msgs::msg::Int32>();
    position_msg->data = position;
    position_pub_->publish(std::move(position_msg));
    
    // 发布角度位置
    auto position_deg_msg = std::make_unique<std_msgs::msg::Float32>();
    position_deg_msg->data = encoderToDegrees(position);
    position_deg_pub_->publish(std::move(position_deg_msg));
    
    // 发布速度
    int32_t velocity = cia402_driver_->getVelocity();
    auto velocity_msg = std::make_unique<std_msgs::msg::Int32>();
    velocity_msg->data = velocity;
    velocity_pub_->publish(std::move(velocity_msg));
    
    // 发布RPM速度
    auto velocity_rpm_msg = std::make_unique<std_msgs::msg::Float32>();
    velocity_rpm_msg->data = velocityToRpm(velocity);
    velocity_rpm_pub_->publish(std::move(velocity_rpm_msg));
    
    // 发布状态字
    uint16_t status = cia402_driver_->getStatusWord();
    auto status_msg = std::make_unique<std_msgs::msg::UInt16>();
    status_msg->data = status;
    status_pub_->publish(std::move(status_msg));
}

int32_t YZMotorNode::degreesToEncoder(double degrees) {
    // 打印转换详情以便调试
    int32_t encoder_value = static_cast<int32_t>(degrees * position_scale_ / 360.0);
    RCLCPP_DEBUG(this->get_logger(), 
                "Converting %.2f degrees to encoder value %d (scale: %.2f)", 
                degrees, encoder_value, position_scale_);
    return encoder_value;
}

double YZMotorNode::encoderToDegrees(int32_t encoder) {
    return (static_cast<double>(encoder) / position_scale_) * 360.0;
}

int32_t YZMotorNode::rpmToVelocity(double rpm) {
    return static_cast<int32_t>(rpm * velocity_scale_);
}

double YZMotorNode::velocityToRpm(int32_t velocity) {
    return static_cast<double>(velocity) / velocity_scale_;
}

void YZMotorNode::setVelocityCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /* request */,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    if (!cia402_driver_) {
        response->success = false;
        response->message = "CiA402 driver not initialized";
        return;
    }
    
    // 每次调用增加20%的速度
    current_profile_velocity_ = static_cast<uint32_t>(current_profile_velocity_ * 1.2);
    bool result = cia402_driver_->setProfileVelocity(current_profile_velocity_);
    
    response->success = result;
    response->message = "Current profile velocity: " + std::to_string(current_profile_velocity_);
    RCLCPP_INFO(this->get_logger(), "Set profile velocity to %d: %s", 
                current_profile_velocity_, result ? "success" : "failed");
}

void YZMotorNode::setAccelerationCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /* request */,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    if (!cia402_driver_) {
        response->success = false;
        response->message = "CiA402 driver not initialized";
        return;
    }
    
    // 每次调用增加20%的加速度
    current_profile_acceleration_ = static_cast<uint32_t>(current_profile_acceleration_ * 1.2);
    bool result = cia402_driver_->setProfileAcceleration(current_profile_acceleration_);
    
    response->success = result;
    response->message = "Current profile acceleration: " + std::to_string(current_profile_acceleration_);
    RCLCPP_INFO(this->get_logger(), "Set profile acceleration to %d: %s", 
                current_profile_acceleration_, result ? "success" : "failed");
}

} // namespace yz_motor_driver

// 主函数
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<yz_motor_driver::YZMotorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
