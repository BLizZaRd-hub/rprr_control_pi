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
    
    // 初始化电机
    if (!initMotor()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize motor");
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
    
    // 添加位置到达发布者
    position_reached_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "position_reached", 10);
        
    // 添加到位检测参数服务
    set_position_reached_params_srv_ = this->create_service<yz_motor_driver::srv::SetPositionReachedParams>(
        "set_position_reached_params",
        std::bind(&YZMotorNode::setPositionReachedParamsCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
                  
    // 如果使用TPDO，注册回调
    if (cia402_driver_) {
        canopen_driver_->registerPDOCallback(1, 
            [this](const std::vector<uint8_t>& data) {
                if (cia402_driver_) {
                    cia402_driver_->handleTPDO(data);
                }
            });
    }
    
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

bool YZMotorNode::initMotor() {
    try {
        // 获取参数
        std::string can_interface = this->get_parameter("can_interface").as_string();
        int node_id = this->get_parameter("node_id").as_int();
        double position_scale = this->get_parameter("position_scale").as_double();
        double velocity_scale = this->get_parameter("velocity_scale").as_double();
        
        // 创建CANopen驱动
        canopen_driver_ = std::make_shared<CANopenDriver>(can_interface, node_id);
        
        // 创建CiA402驱动
        cia402_driver_ = std::make_shared<CiA402Driver>(canopen_driver_);
        
        // 设置编码器分辨率
        cia402_driver_->setEncoderResolution(position_scale);
        
        // 初始化CANopen驱动
        if (!canopen_driver_->init()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize CANopen driver");
            return false;
        }
        
        // 初始化CiA402驱动
        if (!cia402_driver_->init()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize CiA402 driver");
            return false;
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
        
        // 添加位置到达发布者
        position_reached_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "position_reached", 10);
            
        // 添加到位检测参数服务
        set_position_reached_params_srv_ = this->create_service<yz_motor_driver::srv::SetPositionReachedParams>(
            "set_position_reached_params",
            std::bind(&YZMotorNode::setPositionReachedParamsCallback, this,
                      std::placeholders::_1, std::placeholders::_2));
                      
        // 如果使用TPDO，注册回调
        if (cia402_driver_) {
            canopen_driver_->registerPDOCallback(1, 
                [this](const std::vector<uint8_t>& data) {
                    if (cia402_driver_) {
                        cia402_driver_->handleTPDO(data);
                    }
                });
        }
        
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
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Motor initialization failed: %s", e.what());
        return false;
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
    
    (void)request;  // 未使用的参数
    
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
    bool result = cia402_driver_->startHoming(17);  // 使用方法17（CANopen预定义回零方法）
    response->success = result;
    response->message = result ? "Homing started" : "Failed to start homing";
}

void YZMotorNode::positionModeCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    
    (void)request;  // 标记参数为已使用，避免警告
    
    if (!cia402_driver_) {
        response->success = false;
        response->message = "Driver not initialized";
        return;
    }
    
    bool result = cia402_driver_->setOperationMode(OperationMode::PROFILE_POSITION);
    response->success = result;
    response->message = result ? "Set to position mode" : "Failed to set position mode";
    
    RCLCPP_INFO(this->get_logger(), "Set to position mode: %s", result ? "success" : "failed");
}

void YZMotorNode::velocityModeCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    
    (void)request;  // 标记参数为已使用，避免警告
    
    if (!cia402_driver_) {
        response->success = false;
        response->message = "Driver not initialized";
        return;
    }
    
    bool result = cia402_driver_->setOperationMode(OperationMode::PROFILE_VELOCITY);
    response->success = result;
    response->message = result ? "Set to velocity mode" : "Failed to set velocity mode";
    
    RCLCPP_INFO(this->get_logger(), "Set to velocity mode: %s", result ? "success" : "failed");
}

void YZMotorNode::saveParamsCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    (void)request;  // 标记参数为已使用，避免警告
    
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
    
    // 使用标准 future 而不是 .then()
    std::future<void> fut = cia402_driver_->moveToPositionAsync(msg->data);
    
    // 创建一个后台任务来处理完成
    std::thread([this, fut = std::move(fut)]() mutable {
        try {
            fut.get();
            
            // 发布到位消息
            auto reached_msg = std::make_unique<std_msgs::msg::Bool>();
            reached_msg->data = true;
            position_reached_pub_->publish(std::move(reached_msg));
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Motion failed: %s", e.what());
            
            // 发布未到位消息
            auto reached_msg = std::make_unique<std_msgs::msg::Bool>();
            reached_msg->data = false;
            position_reached_pub_->publish(std::move(reached_msg));
        }
    }).detach();
}

void YZMotorNode::positionDegCmdCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    if (!cia402_driver_) {
        return;
    }
    
    // 使用标准 future 而不是 .then()
    std::future<void> fut = cia402_driver_->moveToPositionDegAsync(msg->data);
    
    // 创建一个后台任务来处理完成
    std::thread([this, fut = std::move(fut)]() mutable {
        try {
            fut.get();
            
            // 发布到位消息
            auto reached_msg = std::make_unique<std_msgs::msg::Bool>();
            reached_msg->data = true;
            position_reached_pub_->publish(std::move(reached_msg));
            
            RCLCPP_INFO(this->get_logger(), "Position reached");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Motion failed: %s", e.what());
            
            // 发布未到位消息
            auto reached_msg = std::make_unique<std_msgs::msg::Bool>();
            reached_msg->data = false;
            position_reached_pub_->publish(std::move(reached_msg));
        }
    }).detach();
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
        
        // 发布状态
        auto status_msg = std::make_unique<std_msgs::msg::UInt16>();
        status_msg->data = status;
        status_pub_->publish(std::move(status_msg));
    }
}

int32_t YZMotorNode::degreesToEncoder(double degrees) {
    if (!cia402_driver_) {
        return 0;
    }
    return static_cast<int32_t>(degrees * cia402_driver_->getEncoderResolution() / 360.0);
}

double YZMotorNode::encoderToDegrees(int32_t encoder) {
    if (!cia402_driver_) {
        return 0.0;
    }
    return static_cast<double>(encoder) * 360.0 / cia402_driver_->getEncoderResolution();
}

int32_t YZMotorNode::rpmToVelocity(float rpm) {
    return static_cast<int32_t>(rpm * velocity_scale_);
}

float YZMotorNode::velocityToRpm(int32_t velocity) {
    return static_cast<float>(velocity) / velocity_scale_;
}

void YZMotorNode::setVelocityCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    
    if (!cia402_driver_) {
        response->success = false;
        response->message = "Driver not initialized";
        return;
    }
    
    // 从请求中获取数据
    int new_velocity = static_cast<int>(request->data);
    
    // 设置新的速度
    bool result = cia402_driver_->setProfileVelocity(new_velocity);
    
    // 更新当前值
    if (result) {
        current_profile_velocity_ = new_velocity;
    }
    
    response->success = result;
    response->message = result ? "Profile velocity updated" : "Failed to update profile velocity";
    
    RCLCPP_INFO(this->get_logger(), "Set profile velocity to %d: %s", 
                new_velocity, result ? "success" : "failed");
}

void YZMotorNode::setAccelerationCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    
    if (!cia402_driver_) {
        response->success = false;
        response->message = "Driver not initialized";
        return;
    }
    
    // 从请求中获取数据
    int new_acceleration = static_cast<int>(request->data);
    
    // 设置新的加速度
    bool result = cia402_driver_->setProfileAcceleration(new_acceleration);
    
    // 更新当前值
    if (result) {
        current_profile_acceleration_ = new_acceleration;
    }
    
    response->success = result;
    response->message = result ? "Profile acceleration updated" : "Failed to update profile acceleration";
    
    RCLCPP_INFO(this->get_logger(), "Set profile acceleration to %d: %s", 
                new_acceleration, result ? "success" : "failed");
}

void YZMotorNode::setPositionReachedParamsCallback(
    const std::shared_ptr<yz_motor_driver::srv::SetPositionReachedParams::Request> request,
    std::shared_ptr<yz_motor_driver::srv::SetPositionReachedParams::Response> response) {
    
    if (!cia402_driver_) {
        response->success = false;
        response->message = "Driver not initialized";
        return;
    }
    
    cia402_driver_->setPositionReachedParams(
        request->position_error_threshold,
        request->stable_cycles_required,
        std::chrono::milliseconds(request->timeout_ms));
    
    response->success = true;
    response->message = "Parameters updated successfully";
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
