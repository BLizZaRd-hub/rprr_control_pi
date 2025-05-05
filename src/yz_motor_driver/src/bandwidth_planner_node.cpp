#include "rclcpp/rclcpp.hpp"
#include "yz_motor_driver/can_bandwidth_planner.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/string.hpp"

using namespace yz_motor_driver;

class BandwidthPlannerNode : public rclcpp::Node {
public:
    BandwidthPlannerNode() : Node("bandwidth_planner_node") {
        // 声明参数
        this->declare_parameter("bitrate", 1000000);
        this->declare_parameter("safe_utilization", 0.30);
        this->declare_parameter("num_axes", 4);
        this->declare_parameter("sync_frequency", 1000.0);
        this->declare_parameter("rpdo_dlc", 7);
        this->declare_parameter("tpdo_dlc", 7);
        this->declare_parameter("heartbeat_frequency", 5.0);
        
        // 获取参数
        int bitrate = this->get_parameter("bitrate").as_int();
        double safe_util = this->get_parameter("safe_utilization").as_double();
        int num_axes = this->get_parameter("num_axes").as_int();
        double sync_freq = this->get_parameter("sync_frequency").as_double();
        int rpdo_dlc = this->get_parameter("rpdo_dlc").as_int();
        int tpdo_dlc = this->get_parameter("tpdo_dlc").as_int();
        double heartbeat_freq = this->get_parameter("heartbeat_frequency").as_double();
        
        // 创建规划器
        planner_ = std::make_shared<CANBandwidthPlanner>(bitrate, safe_util);
        
        // 添加SYNC帧
        planner_->addSYNC(sync_freq);
        
        // 添加多轴帧
        planner_->addMultiAxisFrames(1, num_axes, rpdo_dlc, tpdo_dlc, sync_freq, heartbeat_freq);
        
        // 计算利用率
        double utilization = planner_->calculateUtilization();
        RCLCPP_INFO(this->get_logger(), "初始CAN总线占用率: %.1f%%", utilization * 100.0);
        
        // 如果超过安全阈值，尝试自动调整
        if (utilization > safe_util) {
            RCLCPP_WARN(this->get_logger(), "CAN总线占用率超过安全阈值 %.1f%%，尝试自动调整...", safe_util * 100.0);
            
            if (planner_->autoAdjust()) {
                RCLCPP_INFO(this->get_logger(), "自动调整成功，新占用率: %.1f%%", planner_->calculateUtilization() * 100.0);
            } else {
                RCLCPP_ERROR(this->get_logger(), "自动调整失败，请考虑手动降低通信需求或升级总线");
            }
        }
        
        // 创建服务
        plan_service_ = this->create_service<std_srvs::srv::Trigger>(
            "plan_bandwidth",
            std::bind(&BandwidthPlannerNode::planBandwidthCallback, this, 
                      std::placeholders::_1, std::placeholders::_2));
        
        // 创建发布者
        summary_pub_ = this->create_publisher<std_msgs::msg::String>("bandwidth_summary", 10);
        
        // 发布初始摘要
        publishSummary();
    }

private:
    std::shared_ptr<CANBandwidthPlanner> planner_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr plan_service_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr summary_pub_;
    
    void planBandwidthCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        
        (void)request;  // 未使用的参数
        
        // 重新获取参数
        int num_axes = this->get_parameter("num_axes").as_int();
        double sync_freq = this->get_parameter("sync_frequency").as_double();
        int rpdo_dlc = this->get_parameter("rpdo_dlc").as_int();
        int tpdo_dlc = this->get_parameter("tpdo_dlc").as_int();
        double heartbeat_freq = this->get_parameter("heartbeat_frequency").as_double();
        
        // 清除现有帧并重新规划
        planner_->clear();
        planner_->addSYNC(sync_freq);
        planner_->addMultiAxisFrames(1, num_axes, rpdo_dlc, tpdo_dlc, sync_freq, heartbeat_freq);
        
        // 计算利用率
        double utilization = planner_->calculateUtilization();
        
        // 如果超过安全阈值，尝试自动调整
        bool success = true;
        if (utilization > planner_->getSafeUtilization()) {
            success = planner_->autoAdjust();
        }
        
        // 发布摘要
        publishSummary();
        
        // 设置响应
        response->success = success;
        if (success) {
            response->message = "带宽规划成功，当前占用率: " + 
                                std::to_string(planner_->calculateUtilization() * 100.0) + "%";
        } else {
            response->message = "带宽规划失败，占用率超过安全阈值";
        }
    }
    
    void publishSummary() {
        auto msg = std::make_unique<std_msgs::msg::String>();
        msg->data = planner_->getSummary();
        summary_pub_->publish(std::move(msg));
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BandwidthPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
