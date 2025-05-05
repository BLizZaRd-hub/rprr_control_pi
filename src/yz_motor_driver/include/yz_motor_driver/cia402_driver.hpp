#ifndef CIA402_DRIVER_HPP
#define CIA402_DRIVER_HPP

#include "yz_motor_driver/canopen_driver.hpp"
#include <memory>
#include <chrono>

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

class CiA402Driver {
public:
    CiA402Driver(std::shared_ptr<CANopenDriver> canopen);
    ~CiA402Driver();

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

private:
    std::shared_ptr<CANopenDriver> canopen_;
    uint16_t control_word_ = 0;
    uint16_t status_word_ = 0;
    OperationMode operation_mode_ = OperationMode::PROFILE_POSITION;
    bool bit4_high_ = false;  // 跟踪Bit 4状态

    // 状态机辅助函数
    bool transitionToState(CiA402State target_state, std::chrono::milliseconds timeout = std::chrono::milliseconds(1000));
    CiA402State getStateFromStatusWord(uint16_t status_word);
    uint16_t getControlWordForState(CiA402State state);

    // 更新状态字
    bool updateStatusWord();
};

} // namespace yz_motor_driver

#endif // CIA402_DRIVER_HPP
