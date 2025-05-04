#ifndef CIA402_DRIVER_HPP
#define CIA402_DRIVER_HPP

#include "yz_motor_driver/canopen_driver.hpp"
#include <memory>
#include <string>
#include <chrono>

namespace yz_motor_driver {

// CiA 402状态机状态
enum class CiA402State {
    NOT_READY_TO_SWITCH_ON = 0,
    SWITCH_ON_DISABLED = 1,
    READY_TO_SWITCH_ON = 2,
    SWITCHED_ON = 3,
    OPERATION_ENABLED = 4,
    QUICK_STOP_ACTIVE = 5,
    FAULT_REACTION_ACTIVE = 6,
    FAULT = 7,
    UNKNOWN = 8
};

// 操作模式
enum class OperationMode {
    PROFILE_POSITION = 1,
    PROFILE_VELOCITY = 3,
    HOMING = 6,
    INTERPOLATED_POSITION = 7
};

// CiA 402驱动类
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

    // PDO通信方法
    bool enableOperationPDO();
    bool setTargetPositionPDO(int32_t position, bool absolute = true);
    bool setTargetVelocityPDO(int32_t velocity);

    // 操作模式设置
    bool setOperationMode(OperationMode mode);
    OperationMode getOperationMode();
    // int8_t getOperationMode();

    // 位置控制
    bool setTargetPosition(int32_t position, bool absolute = true, bool immediate = true);
    int32_t getCurrentPosition();
    int32_t getTargetPosition();

    // 速度控制
    bool setTargetVelocity(int32_t velocity);
    int32_t getCurrentVelocity();

    // 回零控制
    bool startHoming(uint8_t method);
    bool isHomingComplete();

    // 状态监控
    uint16_t getStatusWord();
    CiA402State getState();
    bool isTargetReached();
    bool isFault();

    // 参数设置
    bool setProfileVelocity(uint32_t velocity);
    bool setProfileAcceleration(uint32_t acceleration);
    bool setGearRatio(uint16_t numerator, uint16_t denominator);
    uint16_t getControlWord();
    int32_t getTargetPosition();
    uint32_t getProfileVelocity();
    uint32_t getProfileAcceleration();

    // 参数保存
    bool saveParameters();

private:
    std::shared_ptr<CANopenDriver> canopen_;
    uint16_t control_word_ = 0;
    uint16_t status_word_ = 0;
    OperationMode operation_mode_ = OperationMode::PROFILE_POSITION;

    // 状态机辅助函数
    bool transitionToState(CiA402State target_state, std::chrono::milliseconds timeout = std::chrono::milliseconds(1000));
    CiA402State getStateFromStatusWord(uint16_t status_word);
    uint16_t getControlWordForState(CiA402State state);

    // 更新状态字
    bool updateStatusWord();
};

} // namespace yz_motor_driver

#endif // CIA402_DRIVER_HPP
