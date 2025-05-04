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
    NO_MODE = 0,           // 添加NO_MODE
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
    bool enableOperationPDO();  // 添加PDO使能方法

    // 状态转换
    bool transitionToState(CiA402State target_state, std::chrono::milliseconds timeout = std::chrono::milliseconds(5000));
    
    // 状态获取
    CiA402State getState();
    CiA402State getStateFromStatusWord(uint16_t status_word);
    uint16_t getControlWordForState(CiA402State state);
    bool updateStatusWord();
    uint16_t getStatusWord();
    uint16_t getControlWord();  // 添加获取控制字方法
    bool isTargetReached();     // 添加目标到达检查
    bool isFault();             // 添加故障检查

    // PDO通信方法
    bool setTargetPositionPDO(int32_t position, bool absolute = true);
    bool setTargetVelocityPDO(int32_t velocity);

    // 操作模式设置
    bool setOperationMode(OperationMode mode);
    OperationMode getOperationMode();

    // 位置控制
    bool setTargetPosition(int32_t position, bool absolute = true, bool immediate = true);
    int32_t getCurrentPosition();
    int32_t getTargetPosition();

    // 速度控制
    bool setTargetVelocity(int32_t velocity);
    int32_t getCurrentVelocity();

    // 其他参数设置
    bool setProfileVelocity(uint32_t velocity);
    uint32_t getProfileVelocity();  // 添加获取速度方法
    bool setProfileAcceleration(uint32_t acceleration);
    uint32_t getProfileAcceleration();  // 添加获取加速度方法
    bool setProfileDeceleration(uint32_t deceleration);
    bool setGearRatio(uint16_t numerator, uint16_t denominator);  // 添加设置齿轮比方法
    bool saveParameters();

    // 回零功能
    bool startHoming(uint8_t method);
    bool isHomingComplete();

private:
    std::shared_ptr<CANopenDriver> canopen_;
    uint16_t control_word_ = 0;
    uint16_t status_word_ = 0;
    OperationMode operation_mode_ = OperationMode::NO_MODE;
};

} // namespace yz_motor_driver

#endif // CIA402_DRIVER_HPP
