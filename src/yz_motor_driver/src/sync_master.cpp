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

namespace yz_motor_driver {

// 获取当前单调时钟纳秒值
static uint64_t now_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}

SyncMaster::SyncMaster(const std::string& interface, uint32_t period_ns, double alpha)
    : interface_(interface),
      period_ns_(period_ns),
      alpha_(alpha),
      phase_err_(0),
      deadline_ns_(0),
      drop_counter_(0),
      bus_load_(0.0),
      running_(false),
      can_socket_(-1) {
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
    
    // 启用硬件时间戳
    int timestamp_flags = SOF_TIMESTAMPING_TX_HARDWARE | 
                          SOF_TIMESTAMPING_RAW_HARDWARE |
                          SOF_TIMESTAMPING_SOFTWARE;
    if (setsockopt(can_socket_, SOL_SOCKET, SO_TIMESTAMPING,
                  &timestamp_flags, sizeof(timestamp_flags)) < 0) {
        RCLCPP_WARN(rclcpp::get_logger("sync_master"), 
                   "Failed to enable hardware timestamping: %s", strerror(errno));
        // 继续执行，但可能无法获取硬件时间戳
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
    
    // 发送帧
    ssize_t nbytes = write(can_socket_, &frame, sizeof(frame));
    if (nbytes != sizeof(frame)) {
        RCLCPP_ERROR(rclcpp::get_logger("sync_master"), 
                    "Failed to send SYNC frame: %s", strerror(errno));
        return false;
    }
    
    // 接收控制消息以获取时间戳
    char control[1024];
    struct iovec iov;
    struct msghdr msg;
    struct cmsghdr *cmsg;
    
    iov.iov_base = &frame;
    iov.iov_len = sizeof(frame);
    
    memset(&msg, 0, sizeof(msg));
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;
    msg.msg_control = control;
    msg.msg_controllen = sizeof(control);
    
    // 等待时间戳消息
    struct timespec timeout;
    timeout.tv_sec = 0;
    timeout.tv_nsec = 5000000;  // 5ms超时
    
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(can_socket_, &read_fds);
    
    int ret = pselect(can_socket_ + 1, &read_fds, NULL, NULL, &timeout, NULL);
    if (ret <= 0) {
        if (ret == 0) {
            RCLCPP_WARN(rclcpp::get_logger("sync_master"), "Timeout waiting for TX timestamp");
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("sync_master"), 
                        "Error in select: %s", strerror(errno));
        }
        return false;
    }
    
    // 读取控制消息
    nbytes = recvmsg(can_socket_, &msg, MSG_ERRQUEUE);
    if (nbytes < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("sync_master"), 
                    "Failed to receive TX timestamp: %s", strerror(errno));
        return false;
    }
    
    // 解析时间戳
    bool found_hw_ts = false;
    for (cmsg = CMSG_FIRSTHDR(&msg); cmsg != NULL; cmsg = CMSG_NXTHDR(&msg, cmsg)) {
        if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SO_TIMESTAMPING) {
            struct timespec *ts = (struct timespec *)CMSG_DATA(cmsg);
            // ts[0] 软件时间戳
            // ts[1] 硬件时间戳 (transformed)
            // ts[2] 原始硬件时间戳
            
            // 优先使用硬件时间戳，如果不可用则使用软件时间戳
            if (ts[2].tv_sec || ts[2].tv_nsec) {
                tx_timestamp = ts[2].tv_sec * 1000000000ULL + ts[2].tv_nsec;
                found_hw_ts = true;
            } else if (ts[0].tv_sec || ts[0].tv_nsec) {
                tx_timestamp = ts[0].tv_sec * 1000000000ULL + ts[0].tv_nsec;
                found_hw_ts = true;
            }
            break;
        }
    }
    
    if (!found_hw_ts) {
        RCLCPP_WARN(rclcpp::get_logger("sync_master"), "No timestamp found in control message");
        return false;
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
    // 设置实时属性
    setRealtimeAttributes();
    
    // 初始化
    uint64_t tx_timestamp = 0;
    struct timespec deadline_ts;
    
    RCLCPP_INFO(rclcpp::get_logger("sync_master"), 
               "SYNC master started with period %u ns", period_ns_);
    
    while (running_) {
        // 等待到达绝对截止时间
        deadline_ts.tv_sec = deadline_ns_ / 1000000000ULL;
        deadline_ts.tv_nsec = deadline_ns_ % 1000000000ULL;
        
        int ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline_ts, NULL);
        if (ret != 0 && ret != EINTR) {
            RCLCPP_ERROR(rclcpp::get_logger("sync_master"), 
                        "clock_nanosleep failed: %s", strerror(ret));
            continue;
        }
        
        // 发送SYNC帧并获取硬件时间戳
        if (!sendSyncAndGetTimestamp(tx_timestamp)) {
            drop_counter_++;
            // 继续下一周期
            deadline_ns_ += period_ns_;
            continue;
        }
        
        // 计算相位误差
        phase_err_ = static_cast<int64_t>(tx_timestamp) - static_cast<int64_t>(deadline_ns_);
        
        // 检查是否需要重新对齐
        if (std::abs(phase_err_) > 50000) {  // >50µs
            RCLCPP_WARN(rclcpp::get_logger("sync_master"), 
                       "SYNC jitter overflow (%ld ns); re-aligning base clock", phase_err_);
            deadline_ns_ = tx_timestamp + period_ns_;
        } else {
            // IIR滤波后修正下一周期的deadline
            deadline_ns_ += period_ns_ - static_cast<int64_t>(alpha_ * phase_err_);
        }
        
        // 更新总线负载估计
        updateBusLoad();
    }
    
    RCLCPP_INFO(rclcpp::get_logger("sync_master"), "SYNC master stopped");
}

} // namespace yz_motor_driver