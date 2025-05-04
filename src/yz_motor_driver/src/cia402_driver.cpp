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
    // 按照状态机顺序使能操作
    return transitionToState(CiA402State::OPERATION_ENABLED);
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
    
    // 设置Bit 7 (Fault Reset)
    control_word_ |= (1 << 7);
    if (!canopen_->writeSDO<uint16_t>(0x6040, 0, control_word_)) {
        return false;
    }
    
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
    // 设置目标位置
    if (!canopen_->writeSDO<int32_t>(0x607A, 0, position)) {
        return false;
    }
    
    // 设置控制字
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
    return canopen_->writeSDO<uint8_t>(0x2614, 0x10, 1);
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
    // 根据CiA 402规范解析状态字
    if ((status_word & 0x4F) == 0x00) {
        return CiA402State::NOT_READY_TO_SWITCH_ON;
    } else if ((status_word & 0x4F) == 0x40) {
        return CiA402State::SWITCH_ON_DISABLED;
    } else if ((status_word & 0x6F) == 0x21) {
        return CiA402State::READY_TO_SWITCH_ON;
    } else if ((status_word & 0x6F) == 0x23) {
        return CiA402State::SWITCHED_ON;
    } else if ((status_word & 0x6F) == 0x27) {
        return CiA402State::OPERATION_ENABLED;
    } else if ((status_word & 0x6F) == 0x07) {
        return CiA402State::QUICK_STOP_ACTIVE;
    } else if ((status_word & 0x4F) == 0x0F) {
        return CiA402State::FAULT_REACTION_ACTIVE;
    } else if ((status_word & 0x4F) == 0x08) {
        return CiA402State::FAULT;
    } else {
        return CiA402State::UNKNOWN;
    }
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
    
    status_word_ = new_status_word;
    return true;
}

bool CiA402Driver::enableOperationPDO() {
    // 使用RPDO1发送控制字和操作模式
    // 控制字0x0F（使能操作）
    // 操作模式取决于当前设置
    uint8_t mode = static_cast<uint8_t>(operation_mode_);
    std::vector<uint8_t> data = {0x0F, 0x00, mode, 0x00, 0x00, 0x00, 0x00, 0x00};
    return canopen_->sendPDO(1, data);
}

bool CiA402Driver::setTargetPositionPDO(int32_t position) {
    // 使用RPDO1发送控制字、操作模式和目标位置
    // 控制字0x1F（使能操作+新设定点）
    // 操作模式1（位置模式）
    uint8_t mode = 1; // 位置模式
    std::vector<uint8_t> data = {
        0x1F, 0x00, mode, 0x00,
        static_cast<uint8_t>(position & 0xFF),
        static_cast<uint8_t>((position >> 8) & 0xFF),
        static_cast<uint8_t>((position >> 16) & 0xFF),
        static_cast<uint8_t>((position >> 24) & 0xFF)
    };
    return canopen_->sendPDO(1, data);
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


} // namespace yz_motor_driver
