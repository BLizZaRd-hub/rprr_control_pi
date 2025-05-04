#include "yz_motor_driver/cia402_driver.hpp"
#include <iostream>
#include <thread>

namespace yz_motor_driver {

CiA402Driver::CiA402Driver(std::shared_ptr<CANopenDriver> canopen)
    : canopen_(canopen) {
}

CiA402Driver::~CiA402Driver() {
    // 尝试禁用操作
    disableOperation();
}

bool CiA402Driver::init() {
    if (!canopen_) {
        std::cerr << "CANopen driver not initialized" << std::endl;
        return false;
    }
    
    // 读取当前状态字
    if (!updateStatusWord()) {
        std::cerr << "Failed to read status word" << std::endl;
        return false;
    }
    
    std::cout << "CiA402 driver initialized, current state: " << static_cast<int>(getState()) << std::endl;
    return true;
}

bool CiA402Driver::enableOperation() {
    // 1. 首先发送故障复位命令
    std::cerr << "Sending fault reset command via SDO" << std::endl;
    if (!canopen_->writeSDO<uint16_t>(0x6040, 0, 0x0080)) {
        std::cerr << "Failed to send fault reset via SDO" << std::endl;
        return false;
    }
    
    // 等待一段时间让驱动器处理故障复位
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 清除故障复位位
    if (!canopen_->writeSDO<uint16_t>(0x6040, 0, 0x0000)) {
        std::cerr << "Failed to clear fault reset bit via SDO" << std::endl;
        return false;
    }
    
    // 再等待一段时间
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 2. 设置操作模式为位置模式
    if (!canopen_->writeSDO<int8_t>(0x6060, 0, 1)) {
        std::cerr << "Failed to set operation mode to position mode" << std::endl;
        return false;
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // 3. 检查当前状态
    updateStatusWord();
    CiA402State current_state = getState();
    std::cerr << "Current state after fault reset: " << static_cast<int>(current_state) 
              << ", status word: 0x" << std::hex << status_word_ << std::dec << std::endl;
    
    // 4. 按照CiA402状态机顺序发送控制字
    
    // 4.1 发送Shutdown命令 (0x0006) - 转到Ready to Switch On状态
    if (!canopen_->writeSDO<uint16_t>(0x6040, 0, 0x0006)) {
        std::cerr << "Failed to send shutdown command via SDO" << std::endl;
        return false;
    }
    
    // 等待状态转换
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    updateStatusWord();
    current_state = getState();
    std::cerr << "After shutdown command, state: " << static_cast<int>(current_state) 
              << ", status word: 0x" << std::hex << status_word_ << std::dec << std::endl;
    
    // 4.2 发送Switch On命令 (0x0007) - 转到Switched On状态
    if (!canopen_->writeSDO<uint16_t>(0x6040, 0, 0x0007)) {
        std::cerr << "Failed to send switch on command via SDO" << std::endl;
        return false;
    }
    
    // 等待状态转换
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    updateStatusWord();
    current_state = getState();
    std::cerr << "After switch on command, state: " << static_cast<int>(current_state) 
              << ", status word: 0x" << std::hex << status_word_ << std::dec << std::endl;
    
    // 4.3 发送Enable Operation命令 (0x000F) - 转到Operation Enabled状态
    if (!canopen_->writeSDO<uint16_t>(0x6040, 0, 0x000F)) {
        std::cerr << "Failed to send enable operation command via SDO" << std::endl;
        return false;
    }
    
    // 等待状态转换
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    updateStatusWord();
    current_state = getState();
    std::cerr << "After enable operation command, state: " << static_cast<int>(current_state) 
              << ", status word: 0x" << std::hex << status_word_ << std::dec << std::endl;
    
    // 5. 检查是否成功使能
    bool success = (current_state == CiA402State::OPERATION_ENABLED);
    std::cerr << "Enable operation via SDO result: " << (success ? "success" : "failed") << std::endl;
    
    // 保存控制字
    control_word_ = 0x000F;
    
    return success;
}

bool CiA402Driver::disableOperation() {
    // 禁用操作
    control_word_ = 0x0;  // 所有位清零
    return canopen_->writeSDO<uint16_t>(0x6040, 0, control_word_);
}

bool CiA402Driver::quickStop() {
    // 快速停止
    control_word_ &= ~(1 << 2);  // 清除Bit 2 (Quick Stop)
    return canopen_->writeSDO<uint16_t>(0x6040, 0, control_word_);
}

bool CiA402Driver::resetFault() {
    // 清除故障
    if (getState() != CiA402State::FAULT) {
        return true;  // 不在故障状态
    }
    
    std::cerr << "Resetting fault..." << std::endl;
    
    // 设置Bit 7 (Fault Reset)
    uint16_t reset_cw = 0x0080;  // 只设置Fault Reset位
    if (!canopen_->writeSDO<uint16_t>(0x6040, 0, reset_cw)) {
        std::cerr << "Failed to write fault reset control word" << std::endl;
        return false;
    }
    
    // 等待一段时间
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 清除Bit 7
    reset_cw = 0x0000;  // 清除所有位
    if (!canopen_->writeSDO<uint16_t>(0x6040, 0, reset_cw)) {
        std::cerr << "Failed to clear fault reset control word" << std::endl;
        return false;
    }
    
    // 等待一段时间
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 检查是否清除了故障
    updateStatusWord();
    CiA402State current_state = getState();
    std::cerr << "After fault reset, state: " << static_cast<int>(current_state) << std::endl;
    
    return current_state != CiA402State::FAULT;
}

bool CiA402Driver::setOperationMode(OperationMode mode) {
    operation_mode_ = mode;
    return canopen_->writeSDO<uint8_t>(0x6060, 0, static_cast<uint8_t>(mode));
}

OperationMode CiA402Driver::getOperationMode() {
    int8_t mode = 0;
    canopen_->readSDO<int8_t>(0x6061, 0, mode);  // 读取当前操作模式显示
    return static_cast<OperationMode>(mode);
}

bool CiA402Driver::setTargetPosition(int32_t position, bool absolute, bool immediate) {
    // 1. 确保电机处于Operation Enabled状态
    CiA402State current_state = getState();
    if (current_state != CiA402State::OPERATION_ENABLED) {
        std::cout << "Motor not in Operation Enabled state. Current state: " 
                  << static_cast<int>(current_state) << std::endl;
        
        // 尝试使能电机
        if (!transitionToState(CiA402State::OPERATION_ENABLED)) {
            std::cerr << "Failed to enable motor operation" << std::endl;
            return false;
        }
    }
    
    // 2. 设置目标位置
    if (!canopen_->writeSDO<int32_t>(0x607A, 0, position)) {
        std::cerr << "Failed to set target position" << std::endl;
        return false;
    }
    
    // 3. 设置控制字
    // 保持使能操作位 (0x000F)
    control_word_ = 0x000F;
    
    if (absolute) {
        control_word_ &= ~(1 << 6);  // 清除Bit 6 (绝对位置模式)
    } else {
        control_word_ |= (1 << 6);   // 设置Bit 6 (相对位置模式)
    }
    
    if (immediate) {
        control_word_ |= (1 << 5);   // 设置Bit 5 (立即执行)
    } else {
        control_word_ &= ~(1 << 5);  // 清除Bit 5 (不立即执行)
    }
    
    // 设置Bit 4 (New Setpoint)
    control_word_ |= (1 << 4);
    
    std::cout << "Setting control word: 0x" << std::hex << control_word_ 
              << std::dec << " for target position: " << position << std::endl;
    
    // 4. 发送控制字
    return canopen_->writeSDO<uint16_t>(0x6040, 0, control_word_);
}

int32_t CiA402Driver::getPosition() {
    static auto last_read_time = std::chrono::steady_clock::now();
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        current_time - last_read_time).count();
    
    // 如果距离上次读取不到100ms，则返回缓存的位置
    static int32_t cached_position = 0;
    if (elapsed < 100) {
        return cached_position;
    }
    
    // 更新时间戳
    last_read_time = current_time;
    
    // 读取位置
    int32_t position = 0;
    if (!canopen_->readSDO<int32_t>(0x6064, 0, position)) {
        std::cerr << "Failed to read position" << std::endl;
        return cached_position;  // 读取失败时返回缓存的位置
    }
    
    cached_position = position;
    return position;
}

bool CiA402Driver::setTargetVelocity(int32_t velocity) {
    // 设置目标速度
    if (!canopen_->writeSDO<int32_t>(0x60FF, 0, velocity)) {
        return false;
    }
    
    return true;
}

int32_t CiA402Driver::getCurrentVelocity() {
    int32_t velocity = 0;
    canopen_->readSDO<int32_t>(0x606C, 0, velocity);
    return velocity;
}

bool CiA402Driver::startHoming(uint8_t method) {
    // 设置回零方法
    if (!canopen_->writeSDO<uint8_t>(0x6098, 0, method)) {
        return false;
    }
    
    // 设置控制字
    control_word_ |= (1 << 4);  // 设置Bit 4 (Start Homing)
    
    return canopen_->writeSDO<uint16_t>(0x6040, 0, control_word_);
}

bool CiA402Driver::isHomingComplete() {
    if (!updateStatusWord()) {
        return false;
    }
    
    // 检查Bit 12 (Homing Attained)
    return (status_word_ & (1 << 12)) != 0;
}

uint16_t CiA402Driver::getStatusWord() {
    updateStatusWord();
    return status_word_;
}

CiA402State CiA402Driver::getState() {
    updateStatusWord();
    return getStateFromStatusWord(status_word_);
}

bool CiA402Driver::isTargetReached() {
    if (!updateStatusWord()) {
        return false;
    }
    
    // 检查Bit 10 (Target Reached)
    return (status_word_ & (1 << 10)) != 0;
}

bool CiA402Driver::isFault() {
    if (!updateStatusWord()) {
        return false;
    }
    
    // 检查Bit 3 (Fault)
    return (status_word_ & (1 << 3)) != 0;
}

bool CiA402Driver::setGearRatio(uint16_t numerator, uint16_t denominator) {
    if (!canopen_->writeSDO<uint16_t>(0x260A, 0x10, numerator)) {
        return false;
    }
    
    return canopen_->writeSDO<uint16_t>(0x260B, 0x10, denominator);
}

uint16_t CiA402Driver::getControlWord() {
    return control_word_;
}

uint32_t CiA402Driver::getProfileVelocity() {
    uint32_t velocity = 0;
    canopen_->readSDO<uint32_t>(0x6081, 0, velocity);
    return velocity;
}

uint32_t CiA402Driver::getProfileAcceleration() {
    uint32_t acceleration = 0;
    canopen_->readSDO<uint32_t>(0x6083, 0, acceleration);
    return acceleration;
}

bool CiA402Driver::transitionToState(CiA402State target_state, std::chrono::milliseconds timeout) {
    // 删除这一行
    // auto start_time = std::chrono::steady_clock::now();
    
    // 增加超时时间
    if (timeout == std::chrono::milliseconds(1000)) {
        timeout = std::chrono::milliseconds(5000);  // 增加到5秒
    }
    
    // 首先检查当前状态
    CiA402State current_state = getState();
    std::cerr << "Current state: " << static_cast<int>(current_state) 
              << ", Target state: " << static_cast<int>(target_state) << std::endl;
    
    // 如果已经处于目标状态，直接返回成功
    if (current_state == target_state) {
        std::cerr << "Already in target state" << std::endl;
        return true;
    }
    
    // 如果处于故障状态，先尝试清除故障
    if (current_state == CiA402State::FAULT) {
        std::cerr << "Device in FAULT state, attempting to reset fault" << std::endl;
        if (!resetFault()) {
            std::cerr << "Failed to reset fault" << std::endl;
            return false;
        }
        // 等待故障清除
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        current_state = getState();
        std::cerr << "After fault reset, state: " << static_cast<int>(current_state) << std::endl;
    }
    
    // 根据CiA402状态机，按顺序执行状态转换
    // 不同的目标状态需要不同的转换路径
    std::vector<CiA402State> transition_path;
    
    switch (target_state) {
        case CiA402State::OPERATION_ENABLED:
            // 完整路径: NOT_READY -> SWITCH_ON_DISABLED -> READY_TO_SWITCH_ON -> SWITCHED_ON -> OPERATION_ENABLED
            if (current_state < CiA402State::READY_TO_SWITCH_ON) {
                transition_path.push_back(CiA402State::READY_TO_SWITCH_ON);
            }
            if (current_state < CiA402State::SWITCHED_ON) {
                transition_path.push_back(CiA402State::SWITCHED_ON);
            }
            transition_path.push_back(CiA402State::OPERATION_ENABLED);
            break;
            
        case CiA402State::SWITCHED_ON:
            if (current_state < CiA402State::READY_TO_SWITCH_ON) {
                transition_path.push_back(CiA402State::READY_TO_SWITCH_ON);
            }
            transition_path.push_back(CiA402State::SWITCHED_ON);
            break;
            
        case CiA402State::READY_TO_SWITCH_ON:
            transition_path.push_back(CiA402State::READY_TO_SWITCH_ON);
            break;
            
        default:
            transition_path.push_back(target_state);
            break;
    }
    
    // 执行状态转换路径
    for (const auto& state : transition_path) {
        std::cerr << "Attempting transition to state: " << static_cast<int>(state) << std::endl;
        
        // 获取此状态对应的控制字
        uint16_t new_control_word = getControlWordForState(state);
        std::cerr << "Setting control word: 0x" << std::hex << new_control_word << std::dec << std::endl;
        
        // 更新控制字并写入
        control_word_ = new_control_word;
        if (!canopen_->writeSDO<uint16_t>(0x6040, 0, control_word_)) {
            std::cerr << "Failed to write control word: 0x" << std::hex << control_word_ << std::dec << std::endl;
            return false;
        }
        
        // 等待状态转换
        auto state_start_time = std::chrono::steady_clock::now();
        bool state_reached = false;
        
        while (std::chrono::steady_clock::now() - state_start_time < std::chrono::milliseconds(1000)) {
            // 等待一段时间再检查状态
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            
            // 更新状态字并检查当前状态
            if (!updateStatusWord()) {
                std::cerr << "Failed to update status word" << std::endl;
                continue;
            }
            
            CiA402State current = getState();
            std::cerr << "Current state: " << static_cast<int>(current) << std::endl;
            
            if (current == state) {
                state_reached = true;
                std::cerr << "Successfully transitioned to state: " << static_cast<int>(state) << std::endl;
                break;
            }
        }
        
        if (!state_reached) {
            std::cerr << "Timeout waiting for transition to state: " << static_cast<int>(state) << std::endl;
            return false;
        }
    }
    
    // 最终检查是否达到目标状态
    CiA402State final_state = getState();
    bool success = (final_state == target_state);
    
    std::cerr << "Final state: " << static_cast<int>(final_state) 
              << ", Target state: " << static_cast<int>(target_state)
              << ", Success: " << (success ? "Yes" : "No") << std::endl;
    
    return success;
}

CiA402State CiA402Driver::getStateFromStatusWord(uint16_t status_word) {
    // 打印状态字以便调试
    std::cerr << "Status word: 0x" << std::hex << status_word << std::dec << std::endl;
    
    // 检查故障状态 (Bit 3)
    if ((status_word & (1 << 3)) != 0) {
        return CiA402State::FAULT;
    }
    
    // 检查故障反应激活状态 (Bit 0 = 0, Bit 1 = 1, Bit 2 = 1, Bit 3 = 0)
    if ((status_word & 0x0F) == 0x06) {
        return CiA402State::FAULT_REACTION_ACTIVE;
    }
    
    // 检查快速停止激活状态 (Bit 5 = 0)
    if ((status_word & (1 << 5)) == 0) {
        return CiA402State::QUICK_STOP_ACTIVE;
    }
    
    // 检查操作使能状态 (Bit 0 = 1, Bit 1 = 1, Bit 2 = 1, Bit 3 = 0, Bit 5 = 1, Bit 6 = 0)
    if ((status_word & 0x6F) == 0x27) {
        return CiA402State::OPERATION_ENABLED;
    }
    
    // 检查已切换开状态 (Bit 0 = 1, Bit 1 = 1, Bit 2 = 0, Bit 3 = 0, Bit 5 = 1, Bit 6 = 0)
    if ((status_word & 0x6F) == 0x23) {
        return CiA402State::SWITCHED_ON;
    }
    
    // 检查准备切换开状态 (Bit 0 = 1, Bit 1 = 0, Bit 2 = 0, Bit 3 = 0, Bit 5 = 1, Bit 6 = 0)
    if ((status_word & 0x6F) == 0x21) {
        return CiA402State::READY_TO_SWITCH_ON;
    }
    
    // 检查切换开禁用状态 (Bit 6 = 1)
    if ((status_word & (1 << 6)) != 0) {
        return CiA402State::SWITCH_ON_DISABLED;
    }
    
    // 非标准状态处理 - 根据你的分析，状态字0x0437可能是一种特殊状态
    // 如果Bit 0,1,2都为1，且Bit 4,5也为1，我们尝试将其解释为Operation Enabled
    if ((status_word & 0x37) == 0x37) {
        std::cerr << "Non-standard status word 0x" << std::hex << status_word 
                  << " interpreted as OPERATION_ENABLED" << std::dec << std::endl;
        return CiA402State::OPERATION_ENABLED;
    }
    
    // 如果无法识别状态，返回未知状态
    std::cerr << "Unknown state from status word: 0x" << std::hex << status_word << std::dec << std::endl;
    return CiA402State::UNKNOWN;
}

uint16_t CiA402Driver::getControlWordForState(CiA402State state) {
    // 保留非状态转换位
    uint16_t cw = control_word_ & 0xFF70;
    
    switch (state) {
        case CiA402State::SWITCH_ON_DISABLED:
            cw |= 0x0006;  // Shutdown command
            break;
        case CiA402State::READY_TO_SWITCH_ON:
            cw |= 0x0006;  // Shutdown command
            break;
        case CiA402State::SWITCHED_ON:
            cw |= 0x0007;  // Switch On command
            break;
        case CiA402State::OPERATION_ENABLED:
            cw |= 0x000F;  // Enable Operation command
            break;
        case CiA402State::QUICK_STOP_ACTIVE:
            cw |= 0x0002;  // Quick Stop command
            break;
        default:
            break;
    }
    
    return cw;
}

bool CiA402Driver::updateStatusWord() {
    // 添加时间检查，避免过于频繁地更新
    static auto last_update_time = std::chrono::steady_clock::now();
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        current_time - last_update_time).count();
    
    // 如果距离上次更新不到100ms，则跳过本次更新
    if (elapsed < 100) {
        return true;  // 返回true表示状态没有问题
    }
    
    // 更新时间戳
    last_update_time = current_time;
    
    // 原有的更新逻辑
    uint16_t status = 0;
    if (!canopen_->readSDO<uint16_t>(0x6041, 0, status)) {
        std::cerr << "Failed to read status word" << std::endl;
        return false;
    }
    
    status_word_ = status;
    return true;
}

bool CiA402Driver::enableOperationPDO() {
    // 1. 首先无条件发送故障复位命令
    std::cerr << "Sending fault reset command via PDO" << std::endl;
    std::vector<uint8_t> fault_reset_data = {0x80, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
    if (!canopen_->sendPDO(1, fault_reset_data)) {
        std::cerr << "Failed to send fault reset PDO" << std::endl;
        return false;
    }
    
    // 等待一段时间让驱动器处理故障复位
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 清除故障复位位
    std::vector<uint8_t> clear_reset_data = {0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
    canopen_->sendPDO(1, clear_reset_data);
    
    // 再等待一段时间
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 2. 检查当前状态
    updateStatusWord();
    CiA402State current_state = getState();
    std::cerr << "Current state after fault reset: " << static_cast<int>(current_state) << std::endl;
    
    // 3. 按照CiA402状态机顺序发送控制字
    
    // 3.1 发送Shutdown命令 (0x06) - 转到Ready to Switch On状态
    std::vector<uint8_t> shutdown_data = {0x06, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
    if (!canopen_->sendPDO(1, shutdown_data)) {
        std::cerr << "Failed to send shutdown PDO" << std::endl;
        return false;
    }
    
    // 等待状态转换
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    updateStatusWord();
    current_state = getState();
    std::cerr << "After shutdown command, state: " << static_cast<int>(current_state) << std::endl;
    
    // 3.2 发送Switch On命令 (0x07) - 转到Switched On状态
    std::vector<uint8_t> switch_on_data = {0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
    if (!canopen_->sendPDO(1, switch_on_data)) {
        std::cerr << "Failed to send switch on PDO" << std::endl;
        return false;
    }
    
    // 等待状态转换
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    updateStatusWord();
    current_state = getState();
    std::cerr << "After switch on command, state: " << static_cast<int>(current_state) << std::endl;
    
    // 3.3 发送Enable Operation命令 (0x0F) - 转到Operation Enabled状态
    std::vector<uint8_t> enable_data = {0x0F, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
    if (!canopen_->sendPDO(1, enable_data)) {
        std::cerr << "Failed to send enable operation PDO" << std::endl;
        return false;
    }
    
    // 等待状态转换
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    updateStatusWord();
    current_state = getState();
    std::cerr << "After enable operation command, state: " << static_cast<int>(current_state) << std::endl;
    
    // 4. 检查是否成功使能
    bool success = (current_state == CiA402State::OPERATION_ENABLED);
    std::cerr << "Enable operation via PDO result: " << (success ? "success" : "failed") << std::endl;
    
    return success;
}

bool CiA402Driver::setTargetPositionPDO(int32_t position, bool absolute) {
    // 使用RPDO1发送控制字、操作模式和目标位置
    
    // 设置控制字，确保基本使能位(0-3)保持为1
    uint16_t ctrl_word = 0x000F;  // 基本使能操作位 (Bit 0,1,2,3 = 1)
    
    // 关键步骤：设置相对/绝对位置模式位
    if (!absolute) {
        ctrl_word |= (1 << 6);  // 设置Bit 6 = 1 (相对位置模式)
    }
    
    // 设置新位置命令位
    ctrl_word |= (1 << 4);  // 设置Bit 4 = 1 (新位置)
    
    // 准备RPDO数据
    std::vector<uint8_t> data = {
        static_cast<uint8_t>(ctrl_word & 0xFF),
        static_cast<uint8_t>((ctrl_word >> 8) & 0xFF),
        0x01,  // 操作模式：位置模式 (1)
        0x00,  // 保留字节
        static_cast<uint8_t>(position & 0xFF),
        static_cast<uint8_t>((position >> 8) & 0xFF),
        static_cast<uint8_t>((position >> 16) & 0xFF),
        static_cast<uint8_t>((position >> 24) & 0xFF)
    };
    
    // 发送RPDO
    bool result = canopen_->sendPDO(1, data);
    
    if (result) {
        // 清除新位置命令位
        ctrl_word &= ~(1 << 4);  // 清除Bit 4
        
        // 准备清除命令位的RPDO数据
        std::vector<uint8_t> clear_data = {
            static_cast<uint8_t>(ctrl_word & 0xFF),
            static_cast<uint8_t>((ctrl_word >> 8) & 0xFF),
            0x01,  // 操作模式：位置模式 (1)
            0x00,  // 保留字节
            static_cast<uint8_t>(position & 0xFF),
            static_cast<uint8_t>((position >> 8) & 0xFF),
            static_cast<uint8_t>((position >> 16) & 0xFF),
            static_cast<uint8_t>((position >> 24) & 0xFF)
        };
        
        // 等待一小段时间
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        // 发送清除命令位的RPDO
        canopen_->sendPDO(1, clear_data);
    }
    
    return result;
}

bool CiA402Driver::setTargetVelocityPDO(int32_t velocity) {
    // 使用RPDO3发送控制字、操作模式和目标速度
    // 控制字0x0F（使能操作）
    // 操作模式3（速度模式）
    uint8_t mode = 3; // 速度模式
    std::vector<uint8_t> data = {
        0x0F, 0x00, mode, 0x00,
        static_cast<uint8_t>(velocity & 0xFF),
        static_cast<uint8_t>((velocity >> 8) & 0xFF),
        static_cast<uint8_t>((velocity >> 16) & 0xFF),
        static_cast<uint8_t>((velocity >> 24) & 0xFF)
    };
    return canopen_->sendPDO(3, data);
}

int32_t CiA402Driver::getTargetPosition() {
    int32_t target_position = 0;
    canopen_->readSDO<int32_t>(0x607A, 0, target_position);
    return target_position;
}

bool CiA402Driver::setProfileVelocity(uint32_t velocity) {
    // 设置轮廓速度
    std::cerr << "Setting profile velocity to " << velocity << std::endl;
    return canopen_->writeSDO<uint32_t>(0x6081, 0, velocity);
}

bool CiA402Driver::setProfileAcceleration(uint32_t acceleration) {
    // 设置轮廓加速度
    std::cerr << "Setting profile acceleration to " << acceleration << std::endl;
    return canopen_->writeSDO<uint32_t>(0x6083, 0, acceleration);
}

bool CiA402Driver::saveParameters() {
    // 保存参数到非易失性存储器
    std::cerr << "Saving parameters to non-volatile memory" << std::endl;
    
    // 使用CANopen对象字典中的存储命令
    // 通常使用对象1010h进行存储操作
    // 写入"save"的ASCII码（0x65766173）作为签名
    uint32_t save_signature = 0x65766173;
    
    // 尝试保存所有参数（子索引1）
    bool result = canopen_->writeSDO<uint32_t>(0x1010, 1, save_signature);
    
    if (!result) {
        std::cerr << "Failed to save parameters" << std::endl;
        return false;
    }
    
    // 等待保存完成
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    std::cerr << "Parameters saved successfully" << std::endl;
    return true;
}

int32_t CiA402Driver::getVelocity() {
    static auto last_read_time = std::chrono::steady_clock::now();
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        current_time - last_read_time).count();
    
    // 如果距离上次读取不到100ms，则返回缓存的速度
    static int32_t cached_velocity = 0;
    if (elapsed < 100) {
        return cached_velocity;
    }
    
    // 更新时间戳
    last_read_time = current_time;
    
    // 读取速度
    int32_t velocity = 0;
    if (!canopen_->readSDO<int32_t>(0x606C, 0, velocity)) {
        std::cerr << "Failed to read velocity" << std::endl;
        return cached_velocity;  // 读取失败时返回缓存的速度
    }
    
    cached_velocity = velocity;
    return velocity;
}

} // namespace yz_motor_driver
