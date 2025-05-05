#include "yz_motor_driver/sync_master_node.hpp"

namespace yz_motor_driver {

SyncMasterNode::SyncMasterNode() : Node("sync_master_node") {
    // 声明参数
    this->declare_parameter("can_interface", "can0");
    this->declare_parameter("sync_period_ns", 1000000);  // 1kHz
    this->declare_parameter("iir_alpha", 0.05);
    
    // 获取参数
    can_interface_ = this->get_parameter("can_interface").as_string();
    sync_period_ns_ = this->get_parameter("sync_period_ns").as_int();
    iir_alpha_ = this->get_parameter("iir_alpha").as_double();
    
    // 创建SYNC主时钟
    sync_master_ = std::make_unique<SyncMaster>(can_interface_, sync_period_ns_, iir_alpha_);
    
    // 创建诊断发布者
    diagnostics_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "can_sync_diagnostics", 10);
    
    // 创建服务
    start_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "start_sync_master",
        std::bind(&SyncMasterNode::startCallback, this, 
                  std::placeholders::_1, std::placeholders::_2));
    
    stop_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "stop_sync_master",
        std::bind(&SyncMasterNode::stopCallback, this, 
                  std::placeholders::_1, std::placeholders::_2));
    
    set_period_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "set_sync_period",
        std::bind(&SyncMasterNode::setPeriodCallback, this, 
                  std::placeholders::_1, std::placeholders::_2));
    
    // 创建诊断定时器 (100ms)
    diagnostics_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&SyncMasterNode::diagnosticsTimerCallback, this));
    
    // 自动启动SYNC主时钟
    if (sync_master_->start()) {
        RCLCPP_INFO(this->get_logger(), "SYNC master started automatically");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to start SYNC master");
    }
}

SyncMasterNode::~SyncMasterNode() {
    // 确保SYNC主时钟停止
    if (sync_master_ && sync_master_->isRunning()) {
        sync_master_->stop();
    }
}

void SyncMasterNode::diagnosticsTimerCallback() {
    if (!sync_master_) {
        return;
    }
    
    // 创建诊断消息
    auto msg = std::make_unique<std_msgs::msg::Float32MultiArray>();
    
    // 设置数据布局
    msg->layout.dim.resize(1);
    msg->layout.dim[0].label = "sync_diagnostics";
    msg->layout.dim[0].size = 3;
    msg->layout.dim[0].stride = 3;
    
    // 填充数据
    msg->data.resize(3);
    msg->data[0] = static_cast<float>(sync_master_->getPhaseError()) / 1000.0f;  // 微秒
    msg->data[1] = static_cast<float>(sync_master_->getBusLoad() * 100.0);       // 百分比
    msg->data[2] = static_cast<float>(sync_master_->getDropCount());             // 丢帧计数
    
    // 发布诊断消息
    diagnostics_pub_->publish(std::move(msg));
}

void SyncMasterNode::startCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    (void)request;  // 未使用的参数
    
    if (!sync_master_) {
        response->success = false;
        response->message = "SYNC master not initialized";
        return;
    }
    
    if (sync_master_->isRunning()) {
        response->success = true;
        response->message = "SYNC master already running";
        return;
    }
    
    bool result = sync_master_->start();
    response->success = result;
    response->message = result ? "SYNC master started" : "Failed to start SYNC master";
}

void SyncMasterNode::stopCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    (void)request;  // 未使用的参数
    
    if (!sync_master_) {
        response->success = false;
        response->message = "SYNC master not initialized";
        return;
    }
    
    if (!sync_master_->isRunning()) {
        response->success = true;
        response->message = "SYNC master already stopped";
        return;
    }
    
    sync_master_->stop();
    response->success = true;
    response->message = "SYNC master stopped";
}

void SyncMasterNode::setPeriodCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    
    if (!sync_master_) {
        response->success = false;
        response->message = "SYNC master not initialized";
        return;
    }
    
    // 使用data字段作为周期值的标志
    // true = 1kHz (1,000,000 ns)
    // false = 500Hz (2,000,000 ns)
    uint32_t new_period = request->data ? 1000000 : 2000000;
    
    // 设置新周期
    sync_master_->setPeriod(new_period);
    
    response->success = true;
    response->message = "SYNC period set to " + std::to_string(new_period) + " ns";
    
    RCLCPP_INFO(this->get_logger(), "SYNC period changed to %u ns", new_period);
}

} // namespace yz_motor_driver

// 主函数
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<yz_motor_driver::SyncMasterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
