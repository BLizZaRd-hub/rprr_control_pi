#include "yz_motor_driver/yz_motor_node.hpp"
#include <iostream>
#include <cmath>

namespace yz_motor_driver {

YZMotorNode::YZMotorNode()
    : Node("yz_motor_node"), last_target_reached_(false) {
    // 声明参数
    this->declare_parameter("can_interface", "can0");
    this->declare_parameter("node_id", 1);
    this->declare_parameter("position_scale", 32768.0);  // 1圈 = 32768个编码器脉冲
    this->declare_parameter("velocity_scale", 10.0);     // 速度值 = rpm / 10
    this->declare_parameter("profile_velocity", 1000);  // 默认速度
    this->declare_parameter("profile_acceleration", 1000);  // 默认加速度
    
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
    
    cia402_driver_ = std::make_shared<CiA402Driver>(canopen_driver_);
    if (!cia402_driver_->init()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize CiA402 driver");
        return;
    }
    
    // 设置电机速度和加速度参数
    if (cia402_driver_) {
        cia402_driver_->setProfileVelocity(current_profile_velocity_);
        cia402_driver_->setProfileAcceleration(current_profile_acceleration_);
        RCLCPP_INFO(this->get_logger(), "Set profile velocity: %d, acceleration: %d", 
                    current_profile_velocity_, current_profile_acceleration_);
    }
    
    // 创建服务
    enable_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "enable",
        std::bind(&YZMotorNode::enableCallback, this, std::placeholders::_1, std::placeholders::_2));
    
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
    
    velocity_cmd_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "velocity_cmd", 10,
        std::bind(&YZMotorNode::velocityCmdCallback, this, std::placeholders::_1));
    
    velocity_rpm_cmd_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "velocity_rpm_cmd", 10,
        std::bind(&YZMotorNode::velocityRpmCmdCallback, this, std::placeholders::_1));
    
    // 创建定时器
    status_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&YZMotorNode::statusTimerCallback, this));
    
    set_velocity_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "set_velocity",
        std::bind(&YZMotorNode::setVelocityCallback, this, std::placeholders::_1, std::placeholders::_2));
    
    set_acceleration_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "set_acceleration",
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
    
    // 使用PDO而不是SDO
    bool result = cia402_driver_->enableOperationPDO();
    response->success = result;
    response->message = result ? "Motor enabled using PDO" : "Failed to enable motor";
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
    
    // 修改这里：将第二个参数设为false表示相对位置模式
    bool result = cia402_driver_->setTargetPositionPDO(position, false);
    RCLCPP_INFO(this->get_logger(), "setTargetPositionPDO result: %s", result ? "success" : "failed");
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

void YZMotorNode::statusTimerCallback() {
    static int counter = 0;
    counter++;
    
    if (!cia402_driver_) {
        return;
    }
    
    // 只有每5次调用才读取状态（减少CAN总线负载）
    if (counter % 5 == 0) {
        // 获取状态字
        uint16_t status = cia402_driver_->getStatusWord();
        
        // 检查限位状态
        bool pos_limit = (status & (1 << 13)) != 0;
        bool neg_limit = (status & (1 << 12)) != 0;
        bool target_reached = (status & (1 << 10)) != 0;
        
        if (pos_limit) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Positive limit reached");
        }
        
        if (neg_limit) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Negative limit reached");
        }
        
        if (target_reached && !last_target_reached_) {
            RCLCPP_INFO(this->get_logger(), "Target position reached");
            
            // 目标到达后，清除Bit 4，为下一个位置命令准备
            cia402_driver_->clearNewSetpointBit();
        }
        
        last_target_reached_ = target_reached;
        
        // 发布状态消息
        auto status_msg = std::make_unique<std_msgs::msg::UInt16>();
        status_msg->data = status;
        status_pub_->publish(std::move(status_msg));
        
        // 其他状态处理...
    }
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
    return static_cast<double>(encoder) * 360.0 / position_scale_;
}

int32_t YZMotorNode::rpmToVelocity(double rpm) {
    return static_cast<int32_t>(rpm * velocity_scale_);
}

double YZMotorNode::velocityToRpm(int32_t velocity) {
    return static_cast<double>(velocity) / velocity_scale_;
}

void YZMotorNode::setVelocityCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    
    if (!cia402_driver_) {
        response->success = false;
        response->message = "Driver not initialized";
        return;
    }
    
    // 使用当前的profile_velocity参数值
    uint32_t velocity = current_profile_velocity_;
    bool result = false;
    
    if (request->data) {
        result = cia402_driver_->setProfileVelocity(velocity);
        
        response->success = result;
        response->message = result ? 
            "Set profile velocity to " + std::to_string(velocity) : 
            "Failed to set profile velocity";
        
        RCLCPP_INFO(this->get_logger(), "Set profile velocity to %d: %s", 
                    velocity, result ? "success" : "failed");
    } else {
        response->success = true;
        response->message = "No action taken";
    }
}

void YZMotorNode::setAccelerationCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    
    if (!cia402_driver_) {
        response->success = false;
        response->message = "Driver not initialized";
        return;
    }
    
    // 使用当前的profile_acceleration参数值
    uint32_t acceleration = current_profile_acceleration_;
    bool result = false;
    
    if (request->data) {
        result = cia402_driver_->setProfileAcceleration(acceleration);
        
        response->success = result;
        response->message = result ? 
            "Set profile acceleration to " + std::to_string(acceleration) : 
            "Failed to set profile acceleration";
        
        RCLCPP_INFO(this->get_logger(), "Set profile acceleration to %d: %s", 
                    acceleration, result ? "success" : "failed");
    } else {
        response->success = true;
        response->message = "No action taken";
    }
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
