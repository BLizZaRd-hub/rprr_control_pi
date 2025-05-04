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
    // 构造函数和析构函数
    CiA402Driver(std::shared_ptr<CANopenDriver> canopen);
    ~CiA402Driver();
    
    // 初始化
    bool init();
    
    // 操作模式设置
    bool setOperationMode(OperationMode mode);
    OperationMode getOperationMode();
    
    // 状态控制
    bool enableOperation();
    bool enableOperationPDO();
    bool disableOperation();
    bool quickStop();
    bool resetFault();
    bool transitionToState(CiA402State target_state, std::chrono::milliseconds timeout = std::chrono::milliseconds(1000));
    
    // 位置控制
    bool setTargetPosition(int32_t position, bool absolute = true, bool immediate = false);
    bool setTargetPositionPDO(int32_t position, bool absolute = true);
    bool setProfileVelocity(uint32_t velocity);
    bool setProfileAcceleration(uint32_t acceleration);
    bool setProfileDeceleration(uint32_t deceleration);
    uint32_t getProfileVelocity();
    uint32_t getProfileAcceleration();
    
    // 速度控制
    bool setTargetVelocity(int32_t velocity);
    bool setTargetVelocityPDO(int32_t velocity);
    int32_t getCurrentVelocity();
    
    // 回零功能
    bool startHoming(uint8_t method);
    bool isHomingComplete();
    
    // 状态获取
    CiA402State getState();
    CiA402State getStateFromStatusWord(uint16_t status_word);
    uint16_t getControlWordForState(CiA402State state);
    bool updateStatusWord();
    uint16_t getStatusWord();
    uint16_t getControlWord();
    bool isTargetReached();
    bool isFault();
    
    // 位置和速度获取
    int32_t getPosition();
    int32_t getVelocity();
    int32_t getTargetPosition();
    
    // 参数保存
    bool saveParameters();
    
    // 齿轮比设置
    bool setGearRatio(uint16_t numerator, uint16_t denominator);
    
private:
    std::shared_ptr<CANopenDriver> canopen_;
    uint16_t status_word_;
    uint16_t control_word_;
    OperationMode operation_mode_;  // Changed from current_mode_ to operation_mode_
    
    // 辅助函数
    bool setControlWord(uint16_t control_word);
    bool setControlWordPDO(uint16_t control_word);
};

} // namespace yz_motor_driver

#endif // CIA402_DRIVER_HPP
