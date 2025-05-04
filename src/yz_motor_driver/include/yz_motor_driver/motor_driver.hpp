#ifndef YZ_MOTOR_DRIVER__MOTOR_DRIVER_HPP_
#define YZ_MOTOR_DRIVER__MOTOR_DRIVER_HPP_

#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <stdexcept> // 用于抛出异常

// SocketCAN 相关头文件 (Linux specific)
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

// ROS 2 相关 (如果需要在类内部使用日志)
#include "rclcpp/rclcpp.hpp"

namespace yz_motor_driver
{

// 定义 CiA 402 状态 (可选，但有助于提高代码可读性)
enum class CiA402State {
    UNKNOWN,
    START, // 非 CiA 标准，用于初始状态
    NOT_READY_TO_SWITCH_ON,
    SWITCH_ON_DISABLED,
    READY_TO_SWITCH_ON,
    SWITCHED_ON,
    OPERATION_ENABLED,
    QUICK_STOP_ACTIVE,
    FAULT_REACTION_ACTIVE,
    FAULT
};

class MotorDriver
{
public:
    /**
     * @brief 构造函数
     * @param node_id 电机 CANopen 节点 ID (1-127)
     * @param can_interface_name CAN 接口名称 (例如 "can0")
     * @param bitrate CAN 总线比特率 (例如 500000)
     * @param logger ROS 2 日志记录器 (可选，用于在类内部打印日志)
     */
    MotorDriver(int node_id,
                const std::string& can_interface_name,
                int bitrate,
                rclcpp::Logger logger = rclcpp::get_logger("MotorDriver")); // 提供默认logger

    /**
     * @brief 析构函数，确保线程和套接字被正确关闭
     */
    ~MotorDriver();

    /**
     * @brief 初始化 CAN 连接并执行电机使能序列
     * @return true 如果成功使能, false 如果失败或超时
     */
    bool init_and_enable();

    /**
     * @brief 发送禁用电机命令
     * @return true 如果命令发送成功 (不保证电机立即禁用)
     */
    bool disable();

    /**
     * @brief 发送绝对位置目标指令
     * @param target_pulses 目标位置 (编码器脉冲数)
     * @return true 如果命令发送成功
     */
    bool set_target_position_absolute(int32_t target_pulses);

    /**
     * @brief 发送相对位置目标指令
     * @param relative_pulses 相对移动量 (编码器脉冲数)
     * @return true 如果命令发送成功
     */
    bool set_target_position_relative(int32_t relative_pulses);

    /**
     * @brief 获取当前缓存的实际位置
     * @return 当前位置 (编码器脉冲数)
     */
    int32_t get_current_position();

    /**
     * @brief 获取当前缓存的状态字
     * @return 当前状态字 (UINT16)
     */
    uint16_t get_current_statusword();

    /**
     * @brief 根据缓存的状态字判断电机是否处于 Operation Enabled 状态
     * @return true 如果已使能
     */
    bool is_enabled();

     /**
     * @brief 获取当前解析出的 CiA 402 状态
     * @return CiA402State 枚举值
     */
    CiA402State get_current_state();

    /**
     * @brief 发送故障复位命令
     * @return true 如果命令发送成功
     */
    bool fault_reset();

private:
    /**
     * @brief 打开并绑定 SocketCAN 套接字
     * @return true 如果成功
     */
    bool open_can_socket();

    /**
     * @brief 关闭 SocketCAN 套接字
     */
    void close_can_socket();

    /**
     * @brief CAN 报文接收线程函数
     */
    void can_receive_thread_func();

    /**
     * @brief 发送 CAN 报文
     * @param frame 要发送的 can_frame
     * @return true 如果发送成功
     */
    bool send_can_frame(const struct can_frame& frame);

    /**
     * @brief 构造并发送 SDO 写入请求 (简化版，只处理 1, 2, 4 字节数据)
     * @param index 对象字典索引
     * @param subindex 对象字典子索引
     * @param data 要写入的数据 (最多 4 字节)
     * @param data_size 要写入的数据字节数 (1, 2, or 4)
     * @return true 如果发送成功
     */
    bool send_sdo_write(uint16_t index, uint8_t subindex, uint32_t data, uint8_t data_size);

     /**
     * @brief 构造并发送 RPDO1 (Controlword, Mode, TargetPosition)
     * @param control_word 控制字
     * @param mode 模式
     * @param target_position 目标位置
     * @return true 如果发送成功
     */
    bool send_rpdo1(uint16_t control_word, uint8_t mode, int32_t target_position);

    /**
     * @brief 构造并发送 NMT 命令
     * @param command NMT 命令代码 (例如 0x01 for Start Node)
     * @return true 如果发送成功
     */
    bool send_nmt(uint8_t command);

    /**
     * @brief 解析接收到的 TPDO1 报文
     * @param frame 接收到的 CAN 报文
     */
    void parse_tpdo1(const struct can_frame& frame);

    /**
     * @brief 解析状态字并返回对应的枚举状态
     * @param status_word 状态字
     * @return CiA402State 枚举值
     */
    CiA402State parse_statusword(uint16_t status_word);

    // --- 成员变量 ---
    int node_id_;                     // 电机节点 ID
    std::string can_interface_name_;  // CAN 接口名
    int bitrate_;                     // CAN 比特率 (注意：SocketCAN 打开时不直接设置比特率，需要外部 ip link 命令设置)
    rclcpp::Logger logger_;           // ROS 2 日志记录器

    int can_socket_;                  // SocketCAN 文件描述符
    std::thread can_receive_thread_;  // CAN 接收线程
    std::atomic<bool> running_;       // 控制线程运行的原子布尔值

    std::mutex state_mutex_;          // 用于保护共享状态变量的互斥锁
    int32_t current_position_;        // 当前缓存的电机位置 (脉冲数)
    uint16_t current_statusword_;     // 当前缓存的状态字
    CiA402State current_state_;       // 当前解析出的 CiA 402 状态

    // CANopen COB-IDs (基于节点 ID 计算)
    uint32_t cob_id_nmt_;             // 0x000
    uint32_t cob_id_sync_;            // 0x080 (可能不需要)
    uint32_t cob_id_emcy_;            // 0x080 + node_id_
    uint32_t cob_id_tpdo1_;           // 0x180 + node_id_
    uint32_t cob_id_rpdo1_;           // 0x200 + node_id_
    uint32_t cob_id_tpdo2_;           // 0x280 + node_id_
    uint32_t cob_id_rpdo2_;           // 0x300 + node_id_
    uint32_t cob_id_tpdo3_;           // 0x380 + node_id_
    uint32_t cob_id_rpdo3_;           // 0x400 + node_id_
    uint32_t cob_id_tpdo4_;           // 0x480 + node_id_
    uint32_t cob_id_rpdo4_;           // 0x500 + node_id_
    uint32_t cob_id_sdo_tx_;          // 0x580 + node_id_ (电机发送响应)
    uint32_t cob_id_sdo_rx_;          // 0x600 + node_id_ (主站发送请求)
};

} // namespace yz_motor_driver

#endif // YZ_MOTOR_DRIVER__MOTOR_DRIVER_HPP_