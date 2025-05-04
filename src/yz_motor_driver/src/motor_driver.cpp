#include "yz_motor_driver/motor_driver.hpp" // 包含我们刚刚创建的头文件
#include <iostream> // 用于一些基本的调试输出 (如果需要)
#include <cstring>  // 用于 memset
#include <system_error> // 用于错误码
#include <poll.h> // 用于 CAN 读取超时

namespace yz_motor_driver
{

// --- 构造函数 ---
MotorDriver::MotorDriver(int node_id,
                         const std::string& can_interface_name,
                         int bitrate, // 注意：bitrate 在这里仅作记录，实际设置需外部 ip link 命令
                         rclcpp::Logger logger) :
    node_id_(node_id),
    can_interface_name_(can_interface_name),
    bitrate_(bitrate), // 记录比特率
    logger_(logger),
    can_socket_(-1),   // 初始化套接字为无效值
    running_(false),
    current_position_(0),
    current_statusword_(0),
    current_state_(CiA402State::UNKNOWN)
{
    // --- 计算 COB-IDs ---
    // 注意：CANopen 标准 ID 是 11 位
    if (node_id < 1 || node_id > 127) {
         RCLCPP_ERROR(logger_, "Node ID %d is out of valid range (1-127)", node_id_);
         throw std::invalid_argument("Node ID out of range");
    }
    cob_id_nmt_ = 0x000;
    cob_id_sync_ = 0x080;
    cob_id_emcy_ = 0x080 + node_id_;
    cob_id_tpdo1_ = 0x180 + node_id_;
    cob_id_rpdo1_ = 0x200 + node_id_;
    cob_id_tpdo2_ = 0x280 + node_id_;
    cob_id_rpdo2_ = 0x300 + node_id_;
    cob_id_tpdo3_ = 0x380 + node_id_;
    cob_id_rpdo3_ = 0x400 + node_id_;
    cob_id_tpdo4_ = 0x480 + node_id_;
    cob_id_rpdo4_ = 0x500 + node_id_;
    cob_id_sdo_tx_ = 0x580 + node_id_; // 电机发送 SDO 响应
    cob_id_sdo_rx_ = 0x600 + node_id_; // 主站发送 SDO 请求

    RCLCPP_INFO(logger_, "MotorDriver created for Node ID %d on %s", node_id_, can_interface_name_.c_str());
}

// --- 析构函数 ---
MotorDriver::~MotorDriver()
{
    running_ = false; // 通知接收线程停止
    if (can_receive_thread_.joinable()) {
        can_receive_thread_.join(); // 等待接收线程结束
        RCLCPP_INFO(logger_, "CAN receive thread joined for Node ID %d.", node_id_);
    }
    close_can_socket(); // 关闭 CAN 套接字
    RCLCPP_INFO(logger_, "MotorDriver for Node ID %d destroyed.", node_id_);
}

// --- 打开并绑定 SocketCAN 套接字 ---
bool MotorDriver::open_can_socket()
{
    if (can_socket_ >= 0) {
        RCLCPP_WARN(logger_, "CAN socket for Node ID %d already open.", node_id_);
        return true; // 已经打开
    }

    // 创建 SocketCAN 套接字 (RAW 协议，需要 root 或特定权限/组)
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
        RCLCPP_ERROR(logger_, "Error creating CAN socket for Node ID %d: %s", node_id_, strerror(errno));
        return false;
    }

    // 获取 CAN 接口索引
    struct ifreq ifr;
    // 使用 strncpy 避免缓冲区溢出
    strncpy(ifr.ifr_name, can_interface_name_.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0'; // 确保空字符结尾
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_ERROR(logger_, "Error getting CAN interface index for %s (Node ID %d): %s",
                     can_interface_name_.c_str(), node_id_, strerror(errno));
        close(can_socket_);
        can_socket_ = -1;
        return false;
    }

    // 准备绑定地址结构
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // 绑定套接字到 CAN 接口
    if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        RCLCPP_ERROR(logger_, "Error binding CAN socket to %s (Node ID %d): %s",
                     can_interface_name_.c_str(), node_id_, strerror(errno));
        close(can_socket_);
        can_socket_ = -1;
        return false;
    }

    RCLCPP_INFO(logger_, "Successfully opened and bound CAN socket for Node ID %d on %s.", node_id_, can_interface_name_.c_str());
    return true;
}

// --- 关闭 SocketCAN 套接字 ---
void MotorDriver::close_can_socket()
{
    if (can_socket_ >= 0) {
        close(can_socket_);
        can_socket_ = -1;
        RCLCPP_INFO(logger_, "Closed CAN socket for Node ID %d.", node_id_);
    }
}

// --- CAN 报文接收线程函数 ---
void MotorDriver::can_receive_thread_func()
{
    struct can_frame frame;
    struct pollfd pfd;
    pfd.fd = can_socket_;
    pfd.events = POLLIN; // 监视可读事件

    RCLCPP_INFO(logger_, "CAN receive thread started for Node ID %d.", node_id_);

    while (running_) {
        // 使用 poll 进行带超时的读取 (例如 100ms 超时)
        int ret = poll(&pfd, 1, 100); // 1 个文件描述符，超时 100ms

        if (ret < 0) {
            // poll 出错
            RCLCPP_ERROR(logger_, "Error polling CAN socket for Node ID %d: %s", node_id_, strerror(errno));
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 避免错误循环过快
            continue;
        } else if (ret == 0) {
            // 超时，没有数据可读，继续循环
            continue;
        } else {
            // 有数据可读 (pfd.revents & POLLIN)
            ssize_t nbytes = read(can_socket_, &frame, sizeof(struct can_frame));

            if (nbytes < 0) {
                RCLCPP_ERROR(logger_, "Error reading from CAN socket for Node ID %d: %s", node_id_, strerror(errno));
                // 可以考虑增加错误计数，如果连续错误次数过多则退出线程
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            if (nbytes < (ssize_t)sizeof(struct can_frame)) {
                RCLCPP_WARN(logger_, "Incomplete CAN frame received for Node ID %d (%ld bytes)", node_id_, nbytes);
                continue;
            }

            // --- 在这里处理接收到的报文 ---
            // 根据 COB-ID 判断报文类型并解析
            // 使用互斥锁保护对共享状态变量的访问

            // 示例：解析 TPDO1
            if (frame.can_id == cob_id_tpdo1_) {
                parse_tpdo1(frame);
            }
            // 示例：解析 SDO 响应 (可以添加更复杂的 SDO 客户端逻辑)
            else if (frame.can_id == cob_id_sdo_tx_) {
                 // 这里可以添加 SDO 响应处理逻辑
                 // 例如，唤醒等待 SDO 响应的线程，或者更新某个状态
                 RCLCPP_DEBUG(logger_, "Received SDO response from Node ID %d", node_id_);
            }
            // 可以添加对 EMCY, Heartbeat 等报文的处理
            else if (frame.can_id == cob_id_emcy_) {
                RCLCPP_WARN(logger_, "Received Emergency message from Node ID %d: Data[0]=0x%02X, Data[1]=0x%02X, ...",
                            node_id_, frame.data[0], frame.data[1]); // 解析具体错误码
            }

        }
    }
    RCLCPP_INFO(logger_, "CAN receive thread stopped for Node ID %d.", node_id_);
}

// --- 发送 CAN 报文 ---
bool MotorDriver::send_can_frame(const struct can_frame& frame)
{
    if (can_socket_ < 0) {
        RCLCPP_ERROR(logger_, "CAN socket not open for Node ID %d.", node_id_);
        return false;
    }

    ssize_t nbytes = write(can_socket_, &frame, sizeof(struct can_frame));
    if (nbytes < 0) {
        RCLCPP_ERROR(logger_, "Error writing to CAN socket for Node ID %d: %s", node_id_, strerror(errno));
        return false;
    }
    if (nbytes < (ssize_t)sizeof(struct can_frame)) {
        RCLCPP_ERROR(logger_, "Incomplete CAN frame written for Node ID %d (%ld bytes)", node_id_, nbytes);
        return false;
    }
    // 可以添加 DEBUG 日志打印发送的报文
    // RCLCPP_DEBUG(logger_, "Sent CAN frame: ID=0x%03X, DLC=%d, Data=[...]", frame.can_id, frame.can_dlc);
    return true;
}


// --- 解析 TPDO1 ---
void MotorDriver::parse_tpdo1(const struct can_frame& frame) {
    if (frame.can_dlc >= 6) { // TPDO1 默认映射了 6 字节 (4字节位置 + 2字节状态字)
        // 解析数据 (小端序 Little-Endian)
        int32_t pos = frame.data[0] | (frame.data[1] << 8) | (frame.data[2] << 16) | (frame.data[3] << 24);
        uint16_t status = frame.data[4] | (frame.data[5] << 8);

        // 使用互斥锁保护共享变量的更新
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_position_ = pos;
        current_statusword_ = status;
        current_state_ = parse_statusword(status); // 更新解析后的状态

        // 可以添加 DEBUG 日志
        // RCLCPP_DEBUG(logger_, "Node %d TPDO1: Pos=%d, Status=0x%04X, State=%d",
        //             node_id_, current_position_, current_statusword_, static_cast<int>(current_state_));

    } else {
        RCLCPP_WARN(logger_, "Received TPDO1 for Node ID %d with incorrect DLC: %d", node_id_, frame.can_dlc);
    }
}

// --- 解析状态字 ---
CiA402State MotorDriver::parse_statusword(uint16_t status_word) {
    // 根据 CiA 402 状态图实现 (简化版)
    if ((status_word & 0x4F) == 0x00) { // Mask: xxxx x0xx x1xx 0000 = 0x4F
        return CiA402State::NOT_READY_TO_SWITCH_ON;
    } else if ((status_word & 0x4F) == 0x40) { // Mask: xxxx x1xx x1xx 0000 = 0x4F
        return CiA402State::SWITCH_ON_DISABLED;
    } else if ((status_word & 0x6F) == 0x21) { // Mask: xxxx x1xx x01x 0001 = 0x6F
        return CiA402State::READY_TO_SWITCH_ON;
    } else if ((status_word & 0x6F) == 0x23) { // Mask: xxxx x1xx x01x 0011 = 0x6F
        return CiA402State::SWITCHED_ON;
    } else if ((status_word & 0x6F) == 0x27) { // Mask: xxxx x1xx x01x 0111 = 0x6F
        return CiA402State::OPERATION_ENABLED;
    } else if ((status_word & 0x6F) == 0x07) { // Mask: xxxx x1xx x00x 0111 = 0x6F
        return CiA402State::QUICK_STOP_ACTIVE;
    } else if ((status_word & 0x4F) == 0x0F) { // Mask: xxxx x0xx x1xx 1111 = 0x4F
        return CiA402State::FAULT_REACTION_ACTIVE; // 通常短暂出现
    } else if ((status_word & 0x4F) == 0x08) { // Mask: xxxx x0xx x1xx 1000 = 0x4F
        return CiA402State::FAULT;
    } else {
        // 添加对状态 0x043x 的特殊处理（根据之前的日志）
        if ((status_word & 0x6F) == 0x34 || (status_word & 0x6F) == 0x37) {
             RCLCPP_WARN(logger_, "Node %d in potentially problematic state: 0x%04X", node_id_, status_word);
             // 可以根据需要返回一个特定状态或 UNKNOWN
        }
        return CiA402State::UNKNOWN;
    }
}


// --- 其他成员函数的实现将在后续添加 ---
// init_and_enable(), disable(), set_target_position_absolute(), ...
// send_sdo_write(), send_rpdo1(), send_nmt() ...
// get_current_position(), get_current_statusword(), is_enabled(), get_current_state(), fault_reset() ...

// --- 故障复位 ---
bool MotorDriver::fault_reset()
{
    RCLCPP_INFO(logger_, "Node %d: Attempting Fault Reset.", node_id_);
    // 发送 Fault Reset 命令 (Bit 7 = 1)
    // 使用 SDO 发送 0x0080 到 0x6040
    if (!send_sdo_write(0x6040, 0, 0x0080, 2)) {
         return false;
    }
    // 等待一小段时间让驱动器处理
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    // 需要再次读取状态字确认故障是否清除，这里简化处理，只发送命令

    // 清除 Reset 位 (Bit 7 = 0)，保持其他位为 0
    // 发送 0x0000 到 0x6040
    return send_sdo_write(0x6040, 0, 0x0000, 2);
}


// --- 初始化 CAN 连接并执行电机使能序列 ---
bool MotorDriver::init_and_enable()
{
    RCLCPP_INFO(logger_, "Node %d: Initializing and enabling...", node_id_);

    // 1. 打开 CAN Socket
    if (!open_can_socket()) {
        return false;
    }

    // 2. 启动 CAN 接收线程
    running_ = true;
    can_receive_thread_ = std::thread(&MotorDriver::can_receive_thread_func, this);

    // --- CANopen 初始化序列 ---
    // 等待接收线程启动并可能收到初始状态
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // a. 检查并尝试清除故障
    if (get_current_state() == CiA402State::FAULT) {
        if (!fault_reset()) {
            RCLCPP_ERROR(logger_, "Node %d: Failed to send Fault Reset command.", node_id_);
            // 可以选择继续尝试，或者直接返回失败
        }
        // 等待故障清除
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        if (get_current_state() == CiA402State::FAULT) {
             RCLCPP_ERROR(logger_, "Node %d: Fault state could not be cleared.", node_id_);
             return false;
        }
    }

    // b. 发送 NMT Start Remote Node 命令，使其进入 Pre-Operational
    if (!send_nmt(0x01)) { // NMT Command: Start Remote Node
         RCLCPP_ERROR(logger_, "Node %d: Failed to send NMT Start command.", node_id_);
         return false;
    }
    // 等待节点启动并可能发送 Boot-up 消息或 Heartbeat
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 等待时间可能需要调整

    // c. 配置操作模式为 Profile Position Mode (1)
    RCLCPP_INFO(logger_, "Node %d: Setting Mode of Operation to Profile Position (1).", node_id_);
    if (!send_sdo_write(0x6060, 0, 1, 1)) { // Index 6060, Sub 0, Data 1 (PP Mode), Size 1 byte
        RCLCPP_ERROR(logger_, "Node %d: Failed to send SDO to set Mode of Operation.", node_id_);
        return false;
    }
    // 需要等待 SDO 响应确认，或者延时后检查模式是否设置成功 (通过 SDO 读取 0x6061)
    // 这里简化，假设 SDO 写入成功后模式即被设置
    std::this_thread::sleep_for(std::chrono::milliseconds(100));


    // --- CiA 402 使能状态机转换 ---
    RCLCPP_INFO(logger_, "Node %d: Starting CiA 402 Enable Sequence...", node_id_);
    auto start_time = std::chrono::steady_clock::now();
    auto timeout = std::chrono::seconds(5); // 设置 5 秒超时

    // 循环直到达到 Operation Enabled 或超时
    while (get_current_state() != CiA402State::OPERATION_ENABLED) {
        // 检查超时
        if (std::chrono::steady_clock::now() - start_time > timeout) {
            RCLCPP_ERROR(logger_, "Node %d: Timeout waiting for Operation Enabled state. Current state: %d (Statusword: 0x%04X)",
                         node_id_, static_cast<int>(get_current_state()), get_current_statusword());
            return false;
        }

        CiA402State current_parsed_state = get_current_state(); // 获取当前状态
        uint16_t control_word_to_send = 0;
        bool send_cmd = false;

        switch (current_parsed_state) {
            case CiA402State::START: // 自定义初始状态
            case CiA402State::NOT_READY_TO_SWITCH_ON:
            case CiA402State::SWITCH_ON_DISABLED:
                // 需要发送 Shutdown 命令 (0x06) 进入 Ready to Switch On
                control_word_to_send = 0x0006;
                send_cmd = true;
                RCLCPP_INFO(logger_, "Node %d: State is [%d]. Sending Shutdown (CW=0x06)...", node_id_, static_cast<int>(current_parsed_state));
                break;

            case CiA402State::READY_TO_SWITCH_ON:
                // 需要发送 Switch On 命令 (0x07) 进入 Switched On
                control_word_to_send = 0x0007;
                send_cmd = true;
                RCLCPP_INFO(logger_, "Node %d: State is READY_TO_SWITCH_ON. Sending Switch On (CW=0x07)...", node_id_);
                break;

            case CiA402State::SWITCHED_ON:
                // 需要发送 Enable Operation 命令 (0x0F) 进入 Operation Enabled
                control_word_to_send = 0x000F;
                send_cmd = true;
                RCLCPP_INFO(logger_, "Node %d: State is SWITCHED_ON. Sending Enable Operation (CW=0x0F)...", node_id_);
                break;

            case CiA402State::OPERATION_ENABLED:
                // 已经使能，无需操作
                RCLCPP_INFO(logger_, "Node %d: Already in OPERATION_ENABLED state.", node_id_);
                send_cmd = false;
                break; // 跳出 switch

            case CiA402State::QUICK_STOP_ACTIVE:
                 // 需要先退出 Quick Stop (通常发送 Enable Operation 0x0F)
                 control_word_to_send = 0x000F;
                 send_cmd = true;
                 RCLCPP_WARN(logger_, "Node %d: State is QUICK_STOP_ACTIVE. Sending Enable Operation (CW=0x0F) to exit...", node_id_);
                 break;

            case CiA402State::FAULT_REACTION_ACTIVE:
                // 通常会自动转换到 Fault，等待即可
                RCLCPP_WARN(logger_, "Node %d: State is FAULT_REACTION_ACTIVE. Waiting...", node_id_);
                send_cmd = false;
                break;

            case CiA402State::FAULT:
                RCLCPP_ERROR(logger_, "Node %d: In FAULT state (Statusword: 0x%04X). Cannot enable.", node_id_, get_current_statusword());
                return false; // 使能失败

            case CiA402State::UNKNOWN:
            default:
                // 未知状态，可能需要先复位或等待有效状态字
                RCLCPP_WARN(logger_, "Node %d: In UNKNOWN state (Statusword: 0x%04X). Waiting for valid status...", node_id_, get_current_statusword());
                // 可以尝试发送一个 Shutdown (0x06) 看看是否能进入已知状态
                control_word_to_send = 0x0006;
                send_cmd = true;
                break;
        }

        if (send_cmd) {
            // 使用 SDO 发送控制字
            if (!send_sdo_write(0x6040, 0, control_word_to_send, 2)) {
                RCLCPP_ERROR(logger_, "Node %d: Failed to send Controlword 0x%04X via SDO.", node_id_, control_word_to_send);
                // 可以选择重试或返回失败
            }
        }

        // 等待一小段时间让状态更新
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 这个延时很重要！

    } // end while

    RCLCPP_INFO(logger_, "Node %d: Successfully enabled! State: OPERATION_ENABLED (Statusword: 0x%04X)", node_id_, get_current_statusword());
    return true;
}


// --- 禁用电机 ---
bool MotorDriver::disable()
{
    RCLCPP_INFO(logger_, "Node %d: Disabling motor...", node_id_);
    // 发送 Disable Voltage 命令 (控制字 0x0000) 通常会回到 Switch On Disabled
    // 或者发送 Shutdown (0x0006) 回到 Ready To Switch On
    // 这里我们发送 Shutdown
    if (!send_sdo_write(0x6040, 0, 0x0006, 2)) {
        RCLCPP_ERROR(logger_, "Node %d: Failed to send Shutdown command via SDO.", node_id_);
        return false;
    }
    // 等待状态变化
    // ... (可以添加状态检查) ...
    return true;
}


// --- 设置绝对位置 ---
bool MotorDriver::set_target_position_absolute(int32_t target_pulses)
{
    if (!is_enabled()) {
        RCLCPP_WARN(logger_, "Node %d: Cannot set target position, motor not enabled.", node_id_);
        return false;
    }

    RCLCPP_INFO(logger_, "Node %d: Setting absolute target position: %d pulses", node_id_, target_pulses);

    // 推荐使用 RPDO1 发送目标位置和触发命令
    // 控制字 0x001F: 保持使能，触发新目标点 (绝对模式 Bit 6=0, Bit 4=1)
    // 模式 1: Profile Position
    if (!send_rpdo1(0x001F, 1, target_pulses)) {
         RCLCPP_ERROR(logger_, "Node %d: Failed to send RPDO1 for absolute position.", node_id_);
         return false;
    }

    // 注意：电机内部会自动将控制字的 Bit 4 复位，或者你需要手动复位
    // 这里简化，不处理复位

    return true;
}

// --- 设置相对位置 ---
bool MotorDriver::set_target_position_relative(int32_t relative_pulses)
{
     if (!is_enabled()) {
        RCLCPP_WARN(logger_, "Node %d: Cannot set target position, motor not enabled.", node_id_);
        return false;
    }

    RCLCPP_INFO(logger_, "Node %d: Setting relative target position: %d pulses", node_id_, relative_pulses);

    // 推荐使用 RPDO1 发送目标位置和触发命令
    // 控制字 0x005F: 保持使能，触发新目标点 (相对模式 Bit 6=1, Bit 4=1)
    // 模式 1: Profile Position
    if (!send_rpdo1(0x005F, 1, relative_pulses)) {
         RCLCPP_ERROR(logger_, "Node %d: Failed to send RPDO1 for relative position.", node_id_);
         return false;
    }

    // 注意：电机内部会自动将控制字的 Bit 4 复位，或者你需要手动复位
    // 这里简化，不处理复位

    return true;
}


} // namespace yz_motor_driver