#include "yz_motor_driver/sync_master.hpp"

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/net_tstamp.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <errno.h>
#include <fcntl.h>

// 在实现文件中包含ROS头文件
#include "rclcpp/rclcpp.hpp"

namespace yz_motor_driver {

// 获取当前单调时钟的纳秒时间戳
uint64_t SyncMaster::now_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}

SyncMaster::SyncMaster(const std::string& interface, 
                       uint32_t period_ns,
                       double alpha,
                       bool use_hw_timestamp)
    : interface_(interface), 
      period_ns_(period_ns), 
      alpha_(alpha),
      phase_err_(0),
      deadline_ns_(0),
      drop_counter_(0),
      bus_load_(0.0),
      running_(false),
      can_socket_(-1),
      use_hw_timestamp_(use_hw_timestamp) {
}

SyncMaster::~SyncMaster() {
    stop();
}

bool SyncMaster::start() {
    if (running_) {
        return true;  // 已经在运行
    }
    
    // 初始化CAN socket
    if (!initSocket()) {
        return false;
    }
    
    // 设置初始deadline为当前时间+周期
    deadline_ns_ = now_ns() + period_ns_;
    
    // 启动同步线程
    running_ = true;
    sync_thread_ = std::thread(&SyncMaster::syncThreadFunc, this);
    
    return true;
}

void SyncMaster::stop() {
    if (!running_) {
        return;  // 已经停止
    }
    
    // 停止线程
    running_ = false;
    if (sync_thread_.joinable()) {
        sync_thread_.join();
    }
    
    // 关闭socket
    closeSocket();
}

void SyncMaster::setPeriod(uint32_t period_ns) {
    period_ns_ = period_ns;
}

void SyncMaster::setAlpha(double alpha) {
    alpha_ = alpha;
}

bool SyncMaster::initSocket() {
    // 创建原始CAN socket
    can_socket_ = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    if (can_socket_ < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("sync_master"), 
                    "Failed to create CAN socket: %s", strerror(errno));
        return false;
    }
    
    // 设置接口名称
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, interface_.c_str(), IFNAMSIZ - 1);
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("sync_master"), 
                    "Failed to get interface index: %s", strerror(errno));
        close(can_socket_);
        can_socket_ = -1;
        return false;
    }
    
    // 绑定socket到CAN接口
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_socket_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("sync_master"), 
                    "Failed to bind socket: %s", strerror(errno));
        close(can_socket_);
        can_socket_ = -1;
        return false;
    }
    
    // 检查是否为MCP251x控制器
    bool is_mcp251x = false;
    char path[256];
    snprintf(path, sizeof(path), "/sys/class/net/%s/device/driver/module/drivers/spi:mcp251x", interface_.c_str());
    if (access(path, F_OK) == 0) {
        is_mcp251x = true;
        RCLCPP_WARN(rclcpp::get_logger("sync_master"), 
                   "Detected MCP251x CAN controller which does not support hardware timestamps");
        use_hw_timestamp_ = false;  // 强制禁用硬件时间戳
    }
    
    // 只有在用户请求且不是MCP251x的情况下才尝试启用硬件时间戳
    if (use_hw_timestamp_ && !is_mcp251x) {
        // 启用硬件时间戳
        int timestamp_flags = SOF_TIMESTAMPING_TX_HARDWARE | 
                              SOF_TIMESTAMPING_RAW_HARDWARE |
                              SOF_TIMESTAMPING_SOFTWARE;
        if (setsockopt(can_socket_, SOL_SOCKET, SO_TIMESTAMPING,
                      &timestamp_flags, sizeof(timestamp_flags)) < 0) {
            RCLCPP_WARN(rclcpp::get_logger("sync_master"), 
                       "Failed to enable hardware timestamping: %s", strerror(errno));
            use_hw_timestamp_ = false;  // 失败时禁用硬件时间戳
        } else {
            RCLCPP_INFO(rclcpp::get_logger("sync_master"), 
                       "Hardware timestamping enabled");
        }
    } else {
        RCLCPP_INFO(rclcpp::get_logger("sync_master"), 
                   "Using software timestamps for SYNC timing");
    }
    
    return true;
}

void SyncMaster::closeSocket() {
    if (can_socket_ >= 0) {
        close(can_socket_);
        can_socket_ = -1;
    }
}

bool SyncMaster::setRealtimeAttributes() {
    // 设置CPU亲和性 (绑定到CPU 2)
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(2, &cpuset);
    
    int rc = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
    if (rc != 0) {
        RCLCPP_WARN(rclcpp::get_logger("sync_master"), 
                   "Failed to set CPU affinity: %s", strerror(rc));
        // 继续执行
    }
    
    // 设置实时调度策略
    struct sched_param param;
    param.sched_priority = 90;  // 高优先级
    
    rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    if (rc != 0) {
        RCLCPP_WARN(rclcpp::get_logger("sync_master"), 
                   "Failed to set real-time scheduling: %s", strerror(rc));
        // 继续执行
    }
    
    return true;
}

bool SyncMaster::sendSyncAndGetTimestamp(uint64_t& tx_timestamp) {
    // 准备SYNC帧 (COB-ID 0x80, DLC 0)
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = 0x080;
    frame.can_dlc = 0;
    
    // 记录发送前的软件时间戳
    uint64_t sw_timestamp = now_ns();
    
    // 发送帧
    ssize_t nbytes = write(can_socket_, &frame, sizeof(frame));
    if (nbytes != sizeof(frame)) {
        RCLCPP_ERROR(rclcpp::get_logger("sync_master"), 
                    "Failed to send SYNC frame: %s", strerror(errno));
        return false;
    }
    
    // 如果不使用硬件时间戳，直接返回软件时间戳
    if (!use_hw_timestamp_) {
        tx_timestamp = sw_timestamp;
        return true;
    }
    
    // 尝试获取硬件时间戳
    char control[1024];
    struct iovec iov;
    struct msghdr msg;
    
    iov.iov_base = &frame;
    iov.iov_len = sizeof(frame);
    
    memset(&msg, 0, sizeof(msg));
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;
    msg.msg_control = control;
    msg.msg_controllen = sizeof(control);
    
    // 等待时间戳消息，但设置较短的超时
    struct timespec timeout;
    timeout.tv_sec = 0;
    timeout.tv_nsec = 100000;  // 100μs超时
    
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(can_socket_, &read_fds);
    
    int ret = pselect(can_socket_ + 1, &read_fds, NULL, NULL, &timeout, NULL);
    if (ret <= 0) {
        // 超时或错误，使用软件时间戳
        static int timeout_count = 0;
        if (timeout_count++ % 1000 == 0) {  // 每1000次只打印一次，减少日志量
            RCLCPP_WARN(rclcpp::get_logger("sync_master"), 
                       "Timeout waiting for TX timestamp, using software timestamp");
        }
        tx_timestamp = sw_timestamp;
        return true;
    }
    
    // 读取控制消息
    nbytes = recvmsg(can_socket_, &msg, MSG_ERRQUEUE);
    if (nbytes < 0) {
        // 错误，使用软件时间戳
        static int error_count = 0;
        if (error_count++ % 1000 == 0) {  // 每1000次只打印一次，减少日志量
            RCLCPP_WARN(rclcpp::get_logger("sync_master"), 
                       "Failed to receive TX timestamp: %s, using software timestamp", 
                       strerror(errno));
        }
        tx_timestamp = sw_timestamp;
        return true;
    }
    
    // 解析时间戳
    bool found_ts = false;
    struct cmsghdr *cmsg;
    for (cmsg = CMSG_FIRSTHDR(&msg); cmsg != NULL; cmsg = CMSG_NXTHDR(&msg, cmsg)) {
        if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SO_TIMESTAMPING) {
            struct timespec *ts = (struct timespec *)CMSG_DATA(cmsg);
            // ts[0] 软件时间戳
            // ts[1] 硬件时间戳 (transformed)
            // ts[2] 原始硬件时间戳
            
            // 优先使用硬件时间戳，如果不可用则使用软件时间戳
            if (ts[2].tv_sec || ts[2].tv_nsec) {
                tx_timestamp = ts[2].tv_sec * 1000000000ULL + ts[2].tv_nsec;
                found_ts = true;
            } else if (ts[0].tv_sec || ts[0].tv_nsec) {
                tx_timestamp = ts[0].tv_sec * 1000000000ULL + ts[0].tv_nsec;
                found_ts = true;
            }
            break;
        }
    }
    
    if (!found_ts) {
        // 没有找到时间戳，使用软件时间戳
        static bool warning_shown = false;
        if (!warning_shown) {
            RCLCPP_WARN(rclcpp::get_logger("sync_master"), 
                       "No timestamp found in control message, using software timestamp");
            warning_shown = true;
        }
        tx_timestamp = sw_timestamp;
    }
    
    return true;
}

void SyncMaster::updateBusLoad() {
    // 简单估算总线负载
    // 假设每个SYNC帧占用约47位 (标准帧开销)
    // 1kHz SYNC = 47,000 bits/s
    // 1Mbps CAN总线 = 约4.7%负载
    
    double frame_bits = 47.0;  // 标准帧开销
    double bits_per_second = frame_bits * (1000000000.0 / period_ns_);
    bus_load_ = bits_per_second / 1000000.0;  // 假设1Mbps总线
}

void SyncMaster::syncThreadFunc() {
    // 设置线程实时属性
    if (!setRealtimeAttributes()) {
        RCLCPP_WARN(rclcpp::get_logger("sync_master"), 
                   "Failed to set realtime attributes for SYNC thread");
    }
    
    // 初始化时间戳和截止时间
    uint64_t now = now_ns();
    deadline_ns_ = now + period_ns_;
    
    // 主循环
    while (running_) {
        // 发送SYNC帧并获取时间戳
        uint64_t tx_timestamp;
        if (!sendSyncAndGetTimestamp(tx_timestamp)) {
            RCLCPP_ERROR(rclcpp::get_logger("sync_master"), 
                        "Failed to send SYNC frame");
            // 短暂休眠后继续
            struct timespec ts;
            ts.tv_sec = 0;
            ts.tv_nsec = 1000000;  // 1ms
            clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, NULL);
            continue;
        }
        
        // 计算相位误差 (当前时间 - 预期时间)
        now = now_ns();
        int64_t phase_error = static_cast<int64_t>(tx_timestamp - deadline_ns_);
        
        // 使用IIR滤波器平滑相位误差
        phase_err_ = static_cast<int64_t>(alpha_ * phase_error + (1.0 - alpha_) * phase_err_);
        
        // 更新下一个截止时间
        deadline_ns_ += period_ns_;
        
        // 如果相位误差过大，重新同步
        if (std::abs(phase_err_) > 50000) {  // 50μs
            deadline_ns_ = now + period_ns_ - phase_err_;
            RCLCPP_DEBUG(rclcpp::get_logger("sync_master"), 
                        "Resynchronizing SYNC clock, phase error: %ld ns", phase_err_);
        }
        
        // 计算需要睡眠的时间
        int64_t sleep_ns = static_cast<int64_t>(deadline_ns_ - now);
        
        // 检查是否需要丢弃一帧
        if (sleep_ns < 0) {
            // 已经错过了截止时间，增加丢帧计数
            drop_counter_++;
            
            if (drop_counter_ % 100 == 1) {  // 每100帧只打印一次，减少日志量
                RCLCPP_WARN(rclcpp::get_logger("sync_master"), 
                           "Missed SYNC deadline by %ld ns, total drops: %u", 
                           -sleep_ns, drop_counter_);
            }
            
            // 重新计算下一个截止时间
            deadline_ns_ = now + period_ns_;
            sleep_ns = period_ns_;
        }
        
        // 使用绝对时间睡眠到下一个截止时间
        struct timespec deadline_ts;
        deadline_ts.tv_sec = deadline_ns_ / 1000000000ULL;
        deadline_ts.tv_nsec = deadline_ns_ % 1000000000ULL;
        
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline_ts, NULL);
        
        // 更新总线负载估计
        updateBusLoad();
    }
}

} // namespace yz_motor_driver


