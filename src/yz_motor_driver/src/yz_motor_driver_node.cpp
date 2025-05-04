#include <chrono>
#include <functional> // for std::bind
#include <memory>     // for std::make_shared
#include <string>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int16.hpp" // 用于发布状态字
#include "std_srvs/srv/trigger.hpp" // 用于使能/禁用/复位服务

#include "yz_motor_driver/motor_driver.hpp" // 包含我们的驱动类
#include "yz_motor_driver/visibility_control.h" // 可见性控制

using namespace std::chrono_literals;
using std::placeholders::_1; // 用于服务回调

namespace yz_motor_driver
{

class YzMotorDriverNode : public rclcpp::Node
{
public:
    YZ_MOTOR_DRIVER_PUBLIC // 使用可见性宏
    explicit YzMotorDriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("yz_motor_driver_node", options)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing YzMotorDriverNode...");

        // --- 参数声明与获取 ---
        this->declare_parameter<std::string>("can_interface", "can0");
        this->declare_parameter<int>("can_bitrate", 500000);
        // 声明一个整数列表参数来指定要控制的电机节点 ID
        // this->declare_parameter<std::vector<long int>>("node_ids", {3}); // 默认只控制节点 3
        this->declare_parameter<std::string>("node_ids_str", "3"); // 声明字符串参数

        can_interface_ = this->get_parameter("can_interface").as_string();
        can_bitrate_ = this->get_parameter("can_bitrate").as_int();
        // std::vector<long int> node_ids_long = this->get_parameter("node_ids").as_integer_array(); // 注释掉旧的
        std::string node_ids_str = this->get_parameter("node_ids_str").as_string(); // 获取字符串

        // 解析逗号分隔的字符串
        std::stringstream ss(node_ids_str);
        std::string segment;
        node_ids_.clear(); // 清空向量
        while(std::getline(ss, segment, ','))
        {
            try {
                int node_id = std::stoi(segment); // 转换为整数
                if (node_id >= 1 && node_id <= 127) {
                    node_ids_.push_back(node_id);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Node ID %d from string '%s' out of range (1-127), skipping.", node_id, segment.c_str());
                }
            } catch (const std::invalid_argument& e) {
                RCLCPP_WARN(this->get_logger(), "Invalid node ID format in string '%s': %s", segment.c_str(), e.what());
            } catch (const std::out_of_range& e) {
                RCLCPP_WARN(this->get_logger(), "Node ID out of range in string '%s': %s", segment.c_str(), e.what());
            }
        }
        // ... 后续检查 node_ids_ 是否为空 ...
        
        // 将 long int 转换为 int
        for (long int id_long : node_ids_long) {
            if (id_long < 1 || id_long > 127) {
                 RCLCPP_WARN(this->get_logger(), "Node ID %ld out of range (1-127), skipping.", id_long);
                 continue;
            }
            node_ids_.push_back(static_cast<int>(id_long));
        }

        if (node_ids_.empty()) {
             RCLCPP_FATAL(this->get_logger(), "No valid node IDs provided. Shutting down.");
             // 在构造函数中不能直接 shutdown，可以通过抛异常或设置状态让外部处理
             throw std::runtime_error("No valid node IDs provided.");
        }

        RCLCPP_INFO(this->get_logger(), "CAN Interface: %s, Bitrate: %d", can_interface_.c_str(), can_bitrate_);
        std::string ids_str;
        for (int id : node_ids_) { ids_str += std::to_string(id) + " "; }
        RCLCPP_INFO(this->get_logger(), "Controlling Node IDs: [%s]", ids_str.c_str());


        // --- 创建 MotorDriver 实例 ---
        for (int node_id : node_ids_) {
            try {
                // 将节点的 logger 传递给 MotorDriver
                drivers_[node_id] = std::make_shared<MotorDriver>(node_id, can_interface_, can_bitrate_, this->get_logger());
                RCLCPP_INFO(this->get_logger(), "Created MotorDriver for Node ID %d", node_id);

                // --- 为每个电机创建话题发布者 ---
                std::string pos_topic = "motor" + std::to_string(node_id) + "/current_position_deg";
                position_publishers_[node_id] = this->create_publisher<std_msgs::msg::Float32>(pos_topic, 10);
                RCLCPP_INFO(this->get_logger(), "Created publisher for %s", pos_topic.c_str());

                std::string status_topic = "motor" + std::to_string(node_id) + "/motor_status";
                status_publishers_[node_id] = this->create_publisher<std_msgs::msg::UInt16>(status_topic, 10);
                 RCLCPP_INFO(this->get_logger(), "Created publisher for %s", status_topic.c_str());

                // --- 为每个电机创建话题订阅者 ---
                std::string abs_cmd_topic = "motor" + std::to_string(node_id) + "/target_position_deg_absolute";
                // 使用 lambda 捕获 node_id
                auto abs_callback = [this, node_id](const std_msgs::msg::Float32::SharedPtr msg) {
                    this->absolute_position_callback(node_id, msg);
                };
                absolute_pos_subscribers_[node_id] = this->create_subscription<std_msgs::msg::Float32>(
                    abs_cmd_topic, 10, abs_callback);
                RCLCPP_INFO(this->get_logger(), "Created subscriber for %s", abs_cmd_topic.c_str());

                std::string rel_cmd_topic = "motor" + std::to_string(node_id) + "/target_position_deg_relative";
                auto rel_callback = [this, node_id](const std_msgs::msg::Float32::SharedPtr msg) {
                    this->relative_position_callback(node_id, msg);
                };
                relative_pos_subscribers_[node_id] = this->create_subscription<std_msgs::msg::Float32>(
                    rel_cmd_topic, 10, rel_callback);
                RCLCPP_INFO(this->get_logger(), "Created subscriber for %s", rel_cmd_topic.c_str());

            } catch (const std::exception& e) {
                 RCLCPP_ERROR(this->get_logger(), "Failed to create MotorDriver for Node ID %d: %s", node_id, e.what());
                 // 可以选择继续创建其他驱动，或者抛出异常终止节点
            }
        }

        // --- 创建服务 ---
        enable_service_ = this->create_service<std_srvs::srv::Trigger>(
            "~/enable_motors", // 使用波浪线表示节点命名空间下的服务
            std::bind(&YzMotorDriverNode::handle_enable_service, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), "Created service ~/enable_motors");

        disable_service_ = this->create_service<std_srvs::srv::Trigger>(
            "~/disable_motors",
            std::bind(&YzMotorDriverNode::handle_disable_service, this, _1, _2));
         RCLCPP_INFO(this->get_logger(), "Created service ~/disable_motors");

        reset_service_ = this->create_service<std_srvs::srv::Trigger>(
            "~/reset_faults",
            std::bind(&YzMotorDriverNode::handle_reset_service, this, _1, _2));
         RCLCPP_INFO(this->get_logger(), "Created service ~/reset_faults");


        // --- 创建状态发布定时器 ---
        // 设置一个定时器，例如每 100ms 发布一次状态
        status_publish_timer_ = this->create_wall_timer(
            100ms, std::bind(&YzMotorDriverNode::publish_status, this));
        RCLCPP_INFO(this->get_logger(), "Created status publishing timer (100ms).");

        // --- 尝试自动使能 (可选) ---
        // 你可以选择在节点启动时自动尝试使能所有电机
        // 或者让用户必须通过服务来使能
        RCLCPP_INFO(this->get_logger(), "Attempting to initialize and enable motors on startup...");
        // 这里简单调用服务处理函数，也可以直接调用 driver 的 init_and_enable
        // 注意：服务回调函数可能需要一些时间，如果电机多，启动会变慢
        // handle_enable_service(nullptr, nullptr); // 暂时注释掉，让用户手动使能

        RCLCPP_INFO(this->get_logger(), "YzMotorDriverNode initialization complete.");
    }

private:
    // --- 话题回调函数 ---
    void absolute_position_callback(int node_id, const std_msgs::msg::Float32::SharedPtr msg)
    {
        if (drivers_.count(node_id)) {
            RCLCPP_INFO(this->get_logger(), "Received absolute target for Node %d: %.2f degrees", node_id, msg->data);
            // 将角度转换为脉冲
            const double pulses_per_revolution = 32768.0;
            const double degrees_per_pulse = 360.0 / pulses_per_revolution;
            int32_t target_pulses = static_cast<int32_t>(msg->data / degrees_per_pulse);

            if (!drivers_[node_id]->set_target_position_absolute(target_pulses)) {
                 RCLCPP_ERROR(this->get_logger(), "Node %d: Failed to send absolute position command.", node_id);
            }
        } else {
             RCLCPP_WARN(this->get_logger(), "Received command for unknown Node ID %d", node_id);
        }
    }

    void relative_position_callback(int node_id, const std_msgs::msg::Float32::SharedPtr msg)
    {
         if (drivers_.count(node_id)) {
            RCLCPP_INFO(this->get_logger(), "Received relative target for Node %d: %.2f degrees", node_id, msg->data);
            // 将角度转换为脉冲增量
            const double pulses_per_revolution = 32768.0;
            const double degrees_per_pulse = 360.0 / pulses_per_revolution;
            int32_t relative_pulses = static_cast<int32_t>(msg->data / degrees_per_pulse);

            if (!drivers_[node_id]->set_target_position_relative(relative_pulses)) {
                 RCLCPP_ERROR(this->get_logger(), "Node %d: Failed to send relative position command.", node_id);
            }
        } else {
             RCLCPP_WARN(this->get_logger(), "Received command for unknown Node ID %d", node_id);
        }
    }

    // --- 服务处理函数 ---
    void handle_enable_service(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request; // 标记 request 未使用
        RCLCPP_INFO(this->get_logger(), "Enable service called.");
        bool all_success = true;
        std::string message = "Enable results: ";
        for (auto const& [node_id, driver] : drivers_) {
            RCLCPP_INFO(this->get_logger(), "Attempting to enable Node %d...", node_id);
            if (driver->init_and_enable()) { // 调用驱动的使能函数
                message += "Node " + std::to_string(node_id) + ": OK. ";
                RCLCPP_INFO(this->get_logger(), "Node %d enabled successfully.", node_id);
            } else {
                message += "Node " + std::to_string(node_id) + ": FAILED. ";
                RCLCPP_ERROR(this->get_logger(), "Node %d failed to enable.", node_id);
                all_success = false;
            }
        }
        response->success = all_success;
        response->message = message;
    }

    void handle_disable_service(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
         (void)request;
         RCLCPP_INFO(this->get_logger(), "Disable service called.");
         bool all_success = true;
         std::string message = "Disable results: ";
         for (auto const& [node_id, driver] : drivers_) {
             if (driver->disable()) { // 调用驱动的禁用函数
                 message += "Node " + std::to_string(node_id) + ": OK. ";
             } else {
                 message += "Node " + std::to_string(node_id) + ": FAILED. ";
                 all_success = false;
             }
         }
         response->success = all_success;
         response->message = message;
    }

     void handle_reset_service(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
         (void)request;
         RCLCPP_INFO(this->get_logger(), "Reset service called.");
         bool all_success = true;
         std::string message = "Reset results: ";
         for (auto const& [node_id, driver] : drivers_) {
             if (driver->fault_reset()) { // 调用驱动的复位函数
                 message += "Node " + std::to_string(node_id) + ": OK. ";
             } else {
                 message += "Node " + std::to_string(node_id) + ": FAILED. ";
                 all_success = false;
             }
         }
         response->success = all_success;
         response->message = message;
    }


    // --- 定时器回调函数 ---
    void publish_status()
    {
        for (auto const& [node_id, driver] : drivers_) {
            // 获取状态
            int32_t current_pulses = driver->get_current_position();
            uint16_t status_word = driver->get_current_statusword();
            // 未来可以添加速度获取: float current_velocity = driver->get_current_velocity();

            // 转换单位
            const double pulses_per_revolution = 32768.0;
            const double degrees_per_pulse = 360.0 / pulses_per_revolution;
            float current_degrees = static_cast<float>(current_pulses * degrees_per_pulse);
            // 速度转换 (如果实现)
            // float current_rpm = current_velocity; // 假设 get_current_velocity 返回 RPM

            // 发布消息
            auto pos_msg = std_msgs::msg::Float32();
            pos_msg.data = current_degrees;
            if (position_publishers_.count(node_id)) {
                position_publishers_[node_id]->publish(pos_msg);
            }

            auto status_msg = std_msgs::msg::UInt16();
            status_msg.data = status_word;
             if (status_publishers_.count(node_id)) {
                status_publishers_[node_id]->publish(status_msg);
            }

            // 发布速度 (如果实现)
            // auto vel_msg = std_msgs::msg::Float32();
            // vel_msg.data = current_rpm;
            // velocity_publishers_[node_id]->publish(vel_msg);
        }
    }

    // --- 成员变量 ---
    std::string can_interface_;
    int can_bitrate_;
    std::vector<int> node_ids_;

    // 使用 map 存储每个 Node ID 对应的驱动实例、发布者和订阅者
    std::map<int, std::shared_ptr<MotorDriver>> drivers_;
    std::map<int, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> position_publishers_;
    std::map<int, rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr> status_publishers_;
    // std::map<int, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> velocity_publishers_; // 未来速度
    std::map<int, rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr> absolute_pos_subscribers_;
    std::map<int, rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr> relative_pos_subscribers_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;

    rclcpp::TimerBase::SharedPtr status_publish_timer_;

};

} // namespace yz_motor_driver

#include "rclcpp_components/register_node_macro.hpp"

// 将这个类注册为一个可以通过 ros2 run ... --ros-args -r __node:=... 启动的组件
// 或者通过 launch 文件中的 ComposableNode 加载
RCLCPP_COMPONENTS_REGISTER_NODE(yz_motor_driver::YzMotorDriverNode)

// 如果你想让它也能直接通过 ros2 run yz_motor_driver yz_motor_driver_node 运行，
// 你需要添加一个 main 函数：
// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<yz_motor_driver::YzMotorDriverNode>());
//   rclcpp::shutdown();
//   return 0;
// }