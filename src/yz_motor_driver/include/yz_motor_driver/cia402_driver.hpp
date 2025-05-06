#ifndef CIA402_DRIVER_HPP
#define CIA402_DRIVER_HPP

#include "yz_motor_driver/canopen_driver.hpp"
#include <memory>
#include <chrono>
#include <functional>
#include <future>
#include <queue>
#include <mutex>

namespace yz_motor_driver {

// CiA402状态枚举
enum class CiA402State {
    NOT_READY_TO_SWITCH_ON,
    SWITCH_ON_DISABLED,
    READY_TO_SWITCH_ON,
    SWITCHED_ON,
    OPERATION_ENABLED,
    QUICK_STOP_ACTIVE,
    FAULT_REACTION_ACTIVE,
    FAULT,
    UNKNOWN  // 添加UNKNOWN状态
};

// 操作模式枚举
enum class OperationMode {
    NO_MODE = 0,
    PROFILE_POSITION = 1,
    VELOCITY = 2,
    PROFILE_VELOCITY = 3,
    TORQUE = 4,
    HOMING = 6,
    CYCLIC_SYNC_POSITION = 8,
    CYCLIC_SYNC_VELOCITY = 9,
    CYCLIC_SYNC_TORQUE = 10
};

// 添加电机移动状态枚举
enum class MotionState {
    IDLE,
    BUSY,
    REACHED,
    TIMEOUT,
    FAULT
};

// 添加命令结构体
struct MotionCommand {
    int32_t target_position;
    bool absolute;
    bool immediate;
    std::promise<void> promise;
    std::chrono::steady_clock::time_point deadline;
};

class CiA402Driver {
public:
    // 构造函数
    CiA402Driver(std::shared_ptr<CANopenDriver> canopen);
    
    // 析构函数
    ~CiA402Driver();
    
    // 设置编码器分辨率
    void setEncoderResolution(double resolution) { encoder_resolution_ = resolution; }
    
    // 获取编码器分辨率
    double getEncoderResolution() const { return encoder_resolution_; }
    
    // 将 handleTPDO 从 private 移到 public
    void handleTPDO(const std::vector<uint8_t>& data);

    // 初始化
    bool init();

    // 状态机控制
    bool enableOperation();
    bool disableOperation();
    bool quickStop();
    bool resetFault();

    // 操作模式设置
    bool setOperationMode(OperationMode mode);
    OperationMode getOperationMode();

    // 位置控制
    bool setTargetPosition(int32_t position, bool absolute = true, bool immediate = true);
    bool setTargetPositionPDO(int32_t position, bool absolute = true);
    int32_t getCurrentPosition();  // 与实现匹配

    // 速度控制
    bool setTargetVelocity(int32_t velocity);
    bool setTargetVelocityPDO(int32_t velocity);  // 添加PDO方法声明
    int32_t getCurrentVelocity();  // 与实现匹配

    // 回零功能
    bool startHoming(uint8_t method);  // 添加回零方法声明
    bool isHomingComplete();  // 添加回零完成检查方法声明

    // PDO操作
    bool enableOperationPDO();  // 添加PDO使能方法声明

    // 状态监控
    uint16_t getStatusWord();
    CiA402State getState();
    bool isTargetReached();
    bool isFault();

    // 参数设置
    bool setProfileVelocity(uint32_t velocity);
    bool setProfileAcceleration(uint32_t acceleration);
    bool setGearRatio(uint16_t numerator, uint16_t denominator);

    // 参数保存
    bool saveParameters();
    
    // 添加清除New Setpoint位的方法
    bool clearNewSetpointBit();

    // 添加以下新方法
    std::future<void> moveToPositionAsync(int32_t position, bool absolute = true, bool immediate = true);
    std::future<void> moveToPositionDegAsync(float position_deg);
    std::future<void> moveRelativeDegAsync(float delta_deg);
    
    // 设置到位检测参数
    void setPositionReachedParams(uint32_t position_error_threshold, 
                                 uint32_t stable_cycles_required,
                                 std::chrono::milliseconds timeout_ms);
    
    // 获取当前移动状态
    MotionState getMotionState() const;
    
    // 取消当前移动
    void cancelCurrentMotion();

private:
    std::shared_ptr<CANopenDriver> canopen_;
    uint16_t control_word_ = 0;
    uint16_t status_word_ = 0;
    OperationMode operation_mode_ = OperationMode::PROFILE_POSITION;
    bool bit4_high_ = false;  // 跟踪Bit 4状态
    
    // 添加编码器分辨率成员变量
    double encoder_resolution_ = 32768.0;  // 默认值：32768脉冲/圈

    // 状态机辅助函数
    bool transitionToState(CiA402State target_state, std::chrono::milliseconds timeout = std::chrono::milliseconds(1000));
    CiA402State getStateFromStatusWord(uint16_t status_word);
    uint16_t getControlWordForState(CiA402State state);

    // 更新状态字
    bool updateStatusWord();

    // 添加以下新成员
    MotionState motion_state_ = MotionState::IDLE;
    std::queue<MotionCommand> command_queue_;
    std::mutex queue_mutex_;
    
    uint32_t position_error_threshold_ = 5;  // 默认5个脉冲的误差阈值
    uint32_t stable_cycles_required_ = 3;    // 默认需要3个周期稳定
    uint32_t stable_cycle_count_ = 0;        // 当前稳定周期计数
    std::chrono::milliseconds timeout_duration_{5000};  // 默认5秒超时
    
    // 处理命令队列
    void processNextCommand();
    
    // 检查到位状态
    void checkPositionReached();
    
    // 设置超时检测
    void setupTimeoutDetection(MotionCommand& cmd);
};

} // namespace yz_motor_driver

#endif // CIA402_DRIVER_HPP
