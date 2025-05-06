#include "yz_motor_driver/cia402_driver.hpp"
#include <iostream>
#include <thread>

namespace yz_motor_driver {

CiA402Driver::CiA402Driver(std::shared_ptr<CANopenDriver> canopen)
    : canopen_(canopen), bit4_high_(false) {
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
    // 直接写入0x000F使能操作（根据手册推荐的启动顺序）
    control_word_ = 0x000F;
    return canopen_->writeSDO<uint16_t>(0x6040, 0, control_word_);
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
    // 设置Bit 7 (Fault Reset)
    control_word_ |= (1 << 7);
    if (!canopen_->writeSDO<uint16_t>(0x6040, 0, control_word_)) {
        return false;
    }
    
    // 等待一段时间
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // 清除Bit 7
    control_word_ &= ~(1 << 7);
    return canopen_->writeSDO<uint16_t>(0x6040, 0, control_word_);
}

bool CiA402Driver::setOperationMode(OperationMode mode) {
    operation_mode_ = mode;
    return canopen_->writeSDO<uint8_t>(0x6060, 0, static_cast<uint8_t>(mode));
}

OperationMode CiA402Driver::getOperationMode() {
    uint8_t mode;
    if (canopen_->readSDO<uint8_t>(0x6061, 0, mode)) {
        return static_cast<OperationMode>(mode);
    }
    return operation_mode_;  // 如果读取失败，返回当前设置的模式
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

int32_t CiA402Driver::getCurrentPosition() {
    int32_t position = 0;
    canopen_->readSDO<int32_t>(0x6064, 0, position);
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

bool CiA402Driver::setProfileVelocity(uint32_t velocity) {
    return canopen_->writeSDO<uint32_t>(0x6081, 0, velocity);
}

bool CiA402Driver::setProfileAcceleration(uint32_t acceleration) {
    return canopen_->writeSDO<uint32_t>(0x6083, 0, acceleration);
}

bool CiA402Driver::setGearRatio(uint16_t numerator, uint16_t denominator) {
    if (!canopen_->writeSDO<uint16_t>(0x260A, 0x10, numerator)) {
        return false;
    }
    
    return canopen_->writeSDO<uint16_t>(0x260B, 0x10, denominator);
}

bool CiA402Driver::saveParameters() {
    // 设置Modbus使能为2
    if (!canopen_->writeSDO<uint16_t>(0x2600, 0, 2)) {
        return false;
    }
    
    // 设置保存标志为5
    if (!canopen_->writeSDO<uint16_t>(0x2614, 0, 5)) {
        return false;
    }
    
    std::cout << "Parameters saved. Please power cycle the motor to apply changes." << std::endl;
    return true;
}

bool CiA402Driver::transitionToState(CiA402State target_state, std::chrono::milliseconds timeout) {
    auto start_time = std::chrono::steady_clock::now();
    
    while (std::chrono::steady_clock::now() - start_time < timeout) {
        // 获取当前状态
        CiA402State current_state = getState();
        
        // 如果已经达到目标状态，返回成功
        if (current_state == target_state) {
            return true;
        }
        
        // 根据当前状态和目标状态设置控制字
        uint16_t new_control_word = getControlWordForState(target_state);
        
        // 如果控制字没有变化，不需要再次写入
        if (new_control_word == control_word_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        
        // 更新控制字并写入
        control_word_ = new_control_word;
        if (!canopen_->writeSDO<uint16_t>(0x6040, 0, control_word_)) {
            return false;
        }
        
        // 等待一段时间
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    // 超时
    std::cerr << "Timeout waiting for state transition to " << static_cast<int>(target_state) << std::endl;
    return false;
}

CiA402State CiA402Driver::getStateFromStatusWord(uint16_t status_word) {
    // 根据状态字位定义判断当前状态
    if ((status_word & 0x004F) == 0x0000) return CiA402State::NOT_READY_TO_SWITCH_ON;
    if ((status_word & 0x004F) == 0x0040) return CiA402State::SWITCH_ON_DISABLED;
    if ((status_word & 0x006F) == 0x0021) return CiA402State::READY_TO_SWITCH_ON;
    if ((status_word & 0x006F) == 0x0023) return CiA402State::SWITCHED_ON;
    if ((status_word & 0x006F) == 0x0027) return CiA402State::OPERATION_ENABLED;
    if ((status_word & 0x006F) == 0x0007) return CiA402State::QUICK_STOP_ACTIVE;
    if ((status_word & 0x004F) == 0x000F) return CiA402State::FAULT_REACTION_ACTIVE;
    if ((status_word & 0x004F) == 0x0008) return CiA402State::FAULT;
    return CiA402State::UNKNOWN;  // 现在UNKNOWN已在枚举中定义
}

uint16_t CiA402Driver::getControlWordForState(CiA402State state) {
    uint16_t cw = control_word_ & 0xFF70;  // 保留非状态转换位
    
    switch (state) {
        case CiA402State::READY_TO_SWITCH_ON:
            cw |= 0x06;  // Shutdown (Bit 1, 2)
            break;
        case CiA402State::SWITCHED_ON:
            cw |= 0x07;  // Switch On (Bit 0, 1, 2)
            break;
        case CiA402State::OPERATION_ENABLED:
            cw |= 0x0F;  // Enable Operation (Bit 0, 1, 2, 3)
            break;
        case CiA402State::QUICK_STOP_ACTIVE:
            cw |= 0x02;  // Quick Stop (Bit 1, ~Bit 2)
            break;
        case CiA402State::SWITCH_ON_DISABLED:
            cw |= 0x00;  // Disable Voltage (Bit 1)
            break;
        default:
            break;
    }
    
    return cw;
}

bool CiA402Driver::updateStatusWord() {
    uint16_t new_status_word = 0;
    if (!canopen_->readSDO<uint16_t>(0x6041, 0, new_status_word)) {
        return false;
    }
    
    // 只有当状态字发生变化时才打印详细信息
    if (status_word_ != new_status_word) {
        status_word_ = new_status_word;
        
        // 使用静态变量控制打印频率
        static bool debug_printed = false;
        if (!debug_printed) {
            debug_printed = true;
            std::cout << "Status Word: 0x" << std::hex << status_word_ << std::dec << std::endl;
            std::cout << "  Ready to Switch On: " << ((status_word_ & 0x0001) ? "Yes" : "No") << std::endl;
            std::cout << "  Switched On: " << ((status_word_ & 0x0002) ? "Yes" : "No") << std::endl;
            std::cout << "  Operation Enabled: " << ((status_word_ & 0x0004) ? "Yes" : "No") << std::endl;
            std::cout << "  Fault: " << ((status_word_ & 0x0008) ? "Yes" : "No") << std::endl;
            std::cout << "  Quick Stop: " << ((status_word_ & 0x0020) ? "No" : "Yes") << std::endl;
            std::cout << "  Switch On Disabled: " << ((status_word_ & 0x0040) ? "Yes" : "No") << std::endl;
            std::cout << "  Target Reached: " << ((status_word_ & 0x0400) ? "Yes" : "No") << std::endl;
        }
    }
    
    return true;
}

bool CiA402Driver::enableOperationPDO() {
    // 使用RPDO1发送控制字和操作模式
    // 控制字0x0F（使能操作）
    uint16_t ctrl_word = 0x000F;  // 使能操作
    uint8_t mode = static_cast<uint8_t>(operation_mode_);
    
    std::vector<uint8_t> data = {
        static_cast<uint8_t>(ctrl_word & 0xFF),
        static_cast<uint8_t>((ctrl_word >> 8) & 0xFF),
        mode, 0x00, 0x00, 0x00, 0x00
    };
    
    std::cout << "Enabling operation via PDO, control word: 0x" 
              << std::hex << ctrl_word << ", mode: " << static_cast<int>(mode) << std::dec << std::endl;
    
    return canopen_->sendPDO(1, data);
}

bool CiA402Driver::setTargetPositionPDO(int32_t position, bool absolute) {
    // 使用RPDO1发送控制字、操作模式和目标位置
    uint16_t ctrl_word = 0x000F;  // 使能操作
    
    // 设置位置模式位
    if (!absolute) {
        ctrl_word |= (1 << 6);  // 设置Bit 6 (相对位置模式)
    }
    
    // 设置immediate位
    ctrl_word |= (1 << 5);
    
    uint8_t mode = 1;  // 位置模式 (PPM)
    
    // 第一步：确保Bit 4为0，同时设置目标位置
    if (bit4_high_) {
        // 如果Bit 4当前为高，先发送一帧将其置低
        ctrl_word &= ~(1 << 4);  // 清除Bit 4
        
        std::vector<uint8_t> reset_data = {
            static_cast<uint8_t>(ctrl_word & 0xFF),
            static_cast<uint8_t>((ctrl_word >> 8) & 0xFF),
            mode,
            static_cast<uint8_t>(position & 0xFF),
            static_cast<uint8_t>((position >> 8) & 0xFF),
            static_cast<uint8_t>((position >> 16) & 0xFF),
            static_cast<uint8_t>((position >> 24) & 0xFF)
        };
        
        if (!canopen_->sendPDO(1, reset_data)) {
            return false;
        }
        
        // 短暂延时确保CAN总线有时间处理
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        bit4_high_ = false;
    }
    
    // 第二步：设置Bit 4为1，产生上升沿
    ctrl_word |= (1 << 4);  // 设置Bit 4
    
    std::vector<uint8_t> data = {
        static_cast<uint8_t>(ctrl_word & 0xFF),
        static_cast<uint8_t>((ctrl_word >> 8) & 0xFF),
        mode,
        static_cast<uint8_t>(position & 0xFF),
        static_cast<uint8_t>((position >> 8) & 0xFF),
        static_cast<uint8_t>((position >> 16) & 0xFF),
        static_cast<uint8_t>((position >> 24) & 0xFF)
    };
    
    bit4_high_ = true;
    return canopen_->sendPDO(1, data);
}

bool CiA402Driver::setTargetVelocityPDO(int32_t velocity) {
    // 使用RPDO3发送控制字、操作模式和目标速度
    uint16_t ctrl_word = 0x000F;  // 使能操作
    uint8_t mode = 3;  // 速度模式 (PVM)
    
    std::vector<uint8_t> data = {
        static_cast<uint8_t>(ctrl_word & 0xFF),
        static_cast<uint8_t>((ctrl_word >> 8) & 0xFF),
        mode, 0x00,
        static_cast<uint8_t>(velocity & 0xFF),
        static_cast<uint8_t>((velocity >> 8) & 0xFF),
        static_cast<uint8_t>((velocity >> 16) & 0xFF),
        static_cast<uint8_t>((velocity >> 24) & 0xFF)
    };
    
    return canopen_->sendPDO(3, data);
}

bool CiA402Driver::clearNewSetpointBit() {
    if (!bit4_high_) {
        return true;  // 已经是低，无需操作
    }
    
    // 获取当前控制字
    uint16_t ctrl_word = control_word_;
    ctrl_word &= ~(1 << 4);  // 清除Bit 4
    
    uint8_t mode = static_cast<uint8_t>(operation_mode_);
    
    std::vector<uint8_t> data = {
        static_cast<uint8_t>(ctrl_word & 0xFF),
        static_cast<uint8_t>((ctrl_word >> 8) & 0xFF),
        mode,
        0, 0, 0, 0  // 保持目标位置不变
    };
    
    bool result = canopen_->sendPDO(1, data);
    if (result) {
        bit4_high_ = false;
    }
    return result;
}

std::future<void> CiA402Driver::moveToPositionAsync(int32_t position, bool absolute, bool immediate) {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    
    MotionCommand cmd;
    cmd.target_position = position;
    cmd.absolute = absolute;
    cmd.immediate = immediate;
    
    std::future<void> future = cmd.promise.get_future();
    
    // 设置超时
    setupTimeoutDetection(cmd);
    
    // 添加到队列
    command_queue_.push(std::move(cmd));
    
    // 如果当前空闲，立即处理
    if (motion_state_ == MotionState::IDLE) {
        processNextCommand();
    }
    
    return future;
}

std::future<void> CiA402Driver::moveToPositionDegAsync(float position_deg) {
    int32_t position = static_cast<int32_t>(position_deg * encoder_resolution_ / 360.0);
    return moveToPositionAsync(position);
}

std::future<void> CiA402Driver::moveRelativeDegAsync(float delta_deg) {
    int32_t delta_position = static_cast<int32_t>(delta_deg * encoder_resolution_ / 360.0);
    return moveToPositionAsync(delta_position, false);
}

void CiA402Driver::setPositionReachedParams(uint32_t position_error_threshold, 
                                          uint32_t stable_cycles_required,
                                          std::chrono::milliseconds timeout_ms) {
    position_error_threshold_ = position_error_threshold;
    stable_cycles_required_ = stable_cycles_required;
    timeout_duration_ = timeout_ms;
}

MotionState CiA402Driver::getMotionState() const {
    return motion_state_;
}

void CiA402Driver::cancelCurrentMotion() {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    
    if (motion_state_ == MotionState::BUSY) {
        // 发送快速停止命令
        control_word_ |= (1 << 8);  // 设置Bit 8 (Quick Stop)
        canopen_->writeSDO<uint16_t>(0x6040, 0, control_word_);
        
        // 清空队列，拒绝所有等待的promise
        while (!command_queue_.empty()) {
            auto& cmd = command_queue_.front();
            cmd.promise.set_exception(std::make_exception_ptr(
                std::runtime_error("Motion cancelled")));
            command_queue_.pop();
        }
        
        motion_state_ = MotionState::IDLE;
    }
}

void CiA402Driver::handleTPDO(const std::vector<uint8_t>& data) {
    // 假设TPDO1包含状态字(2字节)和实际位置(4字节)
    if (data.size() >= 6) {
        // 解析状态字
        uint16_t status = data[0] | (data[1] << 8);
        
        // 解析实际位置
        int32_t actual_position = data[2] | (data[3] << 8) | 
                                 (data[4] << 16) | (data[5] << 24);
        
        // 更新内部状态
        status_word_ = status;
        
        // 如果在BUSY状态，检查是否到位
        if (motion_state_ == MotionState::BUSY) {
            bool target_reached = (status & (1 << 10)) != 0;
            
            if (target_reached) {
                // 检查位置误差
                if (!command_queue_.empty()) {
                    int32_t target = command_queue_.front().target_position;
                    int32_t error = std::abs(actual_position - target);
                    
                    // 修复符号比较警告
                    if (static_cast<uint32_t>(error) <= position_error_threshold_) {
                        stable_cycle_count_++;
                        
                        if (stable_cycle_count_ >= stable_cycles_required_) {
                            // 满足到位条件
                            motion_state_ = MotionState::REACHED;
                            
                            // 完成当前命令
                            auto cmd = std::move(command_queue_.front());
                            command_queue_.pop();
                            cmd.promise.set_value();
                            
                            // 重置计数器
                            stable_cycle_count_ = 0;
                            
                            // 处理下一个命令
                            if (!command_queue_.empty()) {
                                processNextCommand();
                            } else {
                                motion_state_ = MotionState::IDLE;
                            }
                        }
                    } else {
                        // 位置误差过大，重置计数器
                        stable_cycle_count_ = 0;
                    }
                }
            } else {
                // 目标未到达，重置计数器
                stable_cycle_count_ = 0;
            }
        }
    }
}

void CiA402Driver::processNextCommand() {
    if (command_queue_.empty() || motion_state_ != MotionState::IDLE) {
        return;
    }
    
    auto& cmd = command_queue_.front();
    
    // 设置目标位置
    if (!setTargetPosition(cmd.target_position, cmd.absolute, cmd.immediate)) {
        // 设置失败，拒绝promise
        cmd.promise.set_exception(std::make_exception_ptr(
            std::runtime_error("Failed to set target position")));
        command_queue_.pop();
        return;
    }
    
    // 更新状态为BUSY
    motion_state_ = MotionState::BUSY;
    stable_cycle_count_ = 0;
}

void CiA402Driver::setupTimeoutDetection(MotionCommand& cmd) {
    // 计算预期完成时间
    uint32_t profile_velocity;
    if (canopen_->readSDO<uint32_t>(0x6081, 0, profile_velocity)) {
        if (profile_velocity > 0) {
            // 简单估算：位置变化量/速度 + 安全余量
            int32_t current_pos = getCurrentPosition();
            int32_t delta = std::abs(cmd.target_position - current_pos);
            
            // 转换为时间（毫秒）
            auto estimated_time = std::chrono::milliseconds(
                static_cast<int64_t>(1000.0 * delta / profile_velocity * 1.5));  // 1.5倍安全系数
            
            // 设置截止时间
            cmd.deadline = std::chrono::steady_clock::now() + 
                          std::max(estimated_time, timeout_duration_);
        } else {
            // 使用默认超时
            cmd.deadline = std::chrono::steady_clock::now() + timeout_duration_;
        }
    } else {
        // 读取失败，使用默认超时
        cmd.deadline = std::chrono::steady_clock::now() + timeout_duration_;
    }
}

} // namespace yz_motor_driver
