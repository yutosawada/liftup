#include "motor/oriental_motor_controller.hpp"
#include "motor/oriental_motor_constants.hpp"  // 新規インクルード
#include <chrono>  // added for sleep
#include <thread>  // added for sleep


namespace motor_controller {
    
// 新規コード: 位置モードパラメータ用のセッターメソッド実装

// Software Position Limits:
// 仮に、負側: Object 0x6088, 正側: Object 0x6087, サブインデックス: 0
bool OrientalMotorController::SetSoftwarePositionLimits(int32_t negative_limit, int32_t positive_limit) {
    // 負のソフトウェア位置制限設定
    {
        static const uint8_t kSdoDownload4Cmd = 0x23;
        static const uint16_t kSoftwarePosLimitNegObj = 0x6088;
        const uint8_t subindex = 0x00;
        std::vector<uint8_t> request_data = {
            kSdoDownload4Cmd,
            static_cast<uint8_t>(kSoftwarePosLimitNegObj & 0xFF),
            static_cast<uint8_t>((kSoftwarePosLimitNegObj >> 8) & 0xFF),
            subindex,
            static_cast<uint8_t>(negative_limit & 0xFF),
            static_cast<uint8_t>((negative_limit >> 8) & 0xFF),
            static_cast<uint8_t>((negative_limit >> 16) & 0xFF),
            static_cast<uint8_t>((negative_limit >> 24) & 0xFF)
        };
        std::vector<uint8_t> response_data;
        if (!SdoTransaction(request_data, kSdoExpectedResponseDownload, response_data)) {
            RCLCPP_ERROR(logger_, "Failed to set negative software position limit: %d", negative_limit);
            return false;
        }
    }
    // 正のソフトウェア位置制限設定
    {
        static const uint8_t kSdoDownload4Cmd = 0x23;
        static const uint16_t kSoftwarePosLimitPosObj = 0x6087;
        const uint8_t subindex = 0x00;
        std::vector<uint8_t> request_data = {
            kSdoDownload4Cmd,
            static_cast<uint8_t>(kSoftwarePosLimitPosObj & 0xFF),
            static_cast<uint8_t>((kSoftwarePosLimitPosObj >> 8) & 0xFF),
            subindex,
            static_cast<uint8_t>(positive_limit & 0xFF),
            static_cast<uint8_t>((positive_limit >> 8) & 0xFF),
            static_cast<uint8_t>((positive_limit >> 16) & 0xFF),
            static_cast<uint8_t>((positive_limit >> 24) & 0xFF)
        };
        std::vector<uint8_t> response_data;
        if (!SdoTransaction(request_data, kSdoExpectedResponseDownload, response_data)) {
            RCLCPP_ERROR(logger_, "Failed to set positive software position limit: %d", positive_limit);
            return false;
        }
    }
    RCLCPP_INFO(logger_, "Software position limits set: negative=%d, positive=%d", negative_limit, positive_limit);
    return true;
}

bool OrientalMotorController::SetProfileVelocity(uint32_t velocity) {
    static const uint8_t kSdoDownload4Cmd = 0x23;
    static const uint16_t kProfileVelocityObj = 0x6081;
    const uint8_t subindex = 0x00;
    std::vector<uint8_t> request_data = {
        kSdoDownload4Cmd,
        static_cast<uint8_t>(kProfileVelocityObj & 0xFF),
        static_cast<uint8_t>((kProfileVelocityObj >> 8) & 0xFF),
        subindex,
        static_cast<uint8_t>(velocity & 0xFF),
        static_cast<uint8_t>((velocity >> 8) & 0xFF),
        static_cast<uint8_t>((velocity >> 16) & 0xFF),
        static_cast<uint8_t>((velocity >> 24) & 0xFF)
    };
    std::vector<uint8_t> response_data;
    if (!SdoTransaction(request_data, kSdoExpectedResponseDownload, response_data)) {
        RCLCPP_ERROR(logger_, "Failed to set profile velocity: %u", velocity);
        return false;
    }
    RCLCPP_INFO(logger_, "Profile velocity set to: %u", velocity);
    return true;
}

bool OrientalMotorController::SetEndVelocity(uint32_t end_velocity) {
    static const uint8_t kSdoDownload4Cmd = 0x23;
    static const uint16_t kEndVelocityObj = 0x6082;  // 仮定のオブジェクトID
    const uint8_t subindex = 0x00;
    std::vector<uint8_t> request_data = {
        kSdoDownload4Cmd,
        static_cast<uint8_t>(kEndVelocityObj & 0xFF),
        static_cast<uint8_t>((kEndVelocityObj >> 8) & 0xFF),
        subindex,
        static_cast<uint8_t>(end_velocity & 0xFF),
        static_cast<uint8_t>((end_velocity >> 8) & 0xFF),
        static_cast<uint8_t>((end_velocity >> 16) & 0xFF),
        static_cast<uint8_t>((end_velocity >> 24) & 0xFF)
    };
    std::vector<uint8_t> response_data;
    if (!SdoTransaction(request_data, kSdoExpectedResponseDownload, response_data)) {
        RCLCPP_ERROR(logger_, "Failed to set end velocity: %u", end_velocity);
        return false;
    }
    RCLCPP_INFO(logger_, "End velocity set to: %u", end_velocity);
    return true;
}

bool OrientalMotorController::SetProfileAcceleration(uint32_t acceleration) {
    static const uint8_t kSdoDownload4Cmd = 0x23;
    static const uint16_t kProfileAccelObj = 0x6083;
    const uint8_t subindex = 0x00;
    std::vector<uint8_t> request_data = {
        kSdoDownload4Cmd,
        static_cast<uint8_t>(kProfileAccelObj & 0xFF),
        static_cast<uint8_t>((kProfileAccelObj >> 8) & 0xFF),
        subindex,
        static_cast<uint8_t>(acceleration & 0xFF),
        static_cast<uint8_t>((acceleration >> 8) & 0xFF),
        static_cast<uint8_t>((acceleration >> 16) & 0xFF),
        static_cast<uint8_t>((acceleration >> 24) & 0xFF)
    };
    std::vector<uint8_t> response_data;
    if (!SdoTransaction(request_data, kSdoExpectedResponseDownload, response_data)) {
        RCLCPP_ERROR(logger_, "Failed to set profile acceleration: %u", acceleration);
        return false;
    }
    RCLCPP_INFO(logger_, "Profile acceleration set to: %u", acceleration);
    return true;
}

bool OrientalMotorController::SetProfileDeceleration(uint32_t deceleration) {
    static const uint8_t kSdoDownload4Cmd = 0x23;
    static const uint16_t kProfileDecelObj = 0x6084;
    const uint8_t subindex = 0x00;
    std::vector<uint8_t> request_data = {
        kSdoDownload4Cmd,
        static_cast<uint8_t>(kProfileDecelObj & 0xFF),
        static_cast<uint8_t>((kProfileDecelObj >> 8) & 0xFF),
        subindex,
        static_cast<uint8_t>(deceleration & 0xFF),
        static_cast<uint8_t>((deceleration >> 8) & 0xFF),
        static_cast<uint8_t>((deceleration >> 16) & 0xFF),
        static_cast<uint8_t>((deceleration >> 24) & 0xFF)
    };
    std::vector<uint8_t> response_data;
    if (!SdoTransaction(request_data, kSdoExpectedResponseDownload, response_data)) {
        RCLCPP_ERROR(logger_, "Failed to set profile deceleration: %u", deceleration);
        return false;
    }
    RCLCPP_INFO(logger_, "Profile deceleration set to: %u", deceleration);
    return true;
}

bool OrientalMotorController::SetQuickStopDeceleration(uint32_t quick_stop_deceleration) {
    static const uint8_t kSdoDownload4Cmd = 0x23;
    static const uint16_t kQuickStopDecelObj = 0x6085;
    const uint8_t subindex = 0x00;
    std::vector<uint8_t> request_data = {
        kSdoDownload4Cmd,
        static_cast<uint8_t>(kQuickStopDecelObj & 0xFF),
        static_cast<uint8_t>((kQuickStopDecelObj >> 8) & 0xFF),
        subindex,
        static_cast<uint8_t>(quick_stop_deceleration & 0xFF),
        static_cast<uint8_t>((quick_stop_deceleration >> 8) & 0xFF),
        static_cast<uint8_t>((quick_stop_deceleration >> 16) & 0xFF),
        static_cast<uint8_t>((quick_stop_deceleration >> 24) & 0xFF)
    };
    std::vector<uint8_t> response_data;
    if (!SdoTransaction(request_data, kSdoExpectedResponseDownload, response_data)) {
        RCLCPP_ERROR(logger_, "Failed to set quick stop deceleration: %u", quick_stop_deceleration);
        return false;
    }
    RCLCPP_INFO(logger_, "Quick stop deceleration set to: %u", quick_stop_deceleration);
    return true;
}

bool OrientalMotorController::MonitorPositionDriveProfileOperationReady() {
    const int maxIterations = 50; // 50 iterations x 20ms = 1 second
    uint16_t status = 0;
    for (int i = 0; i < maxIterations; i++) {
        if (!GetMotorStatus(&status)) {
            RCLCPP_ERROR(logger_, "MonitorPositionDriveProfileOperationReady: GetMotorStatus failed");
            return false;
        }
        // Assuming the 'Drive profile operation ready' flag is represented by bit 0 being 1.
        if (status & 0x0001) {
            RCLCPP_INFO(logger_, "Drive profile operation ready detected");
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    RCLCPP_ERROR(logger_, "MonitorPositionDriveProfileOperationReady: Drive profile operation ready not detected within 1 second");
    return false;
}


}