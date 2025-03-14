#include "motor/oriental_motor_controller.hpp"
#include "motor/oriental_motor_constants.hpp"  // 新規インクルード
#include <chrono>  // added for sleep
#include <thread>  // added for sleep

namespace motor_controller {

bool OrientalMotorController::PerformHomingProcedure() {

    if (!SetHomingAcceleration(homing_param_.acceleration)) {
        RCLCPP_ERROR(logger_, "PerformHomingProcedure: SetHomingAcceleration failed");
        return false;
    }
    
    if (!SetHomingSpeeds(homing_param_.speed_serch_for_switch, homing_param_.speed_serch_for_zero)) {
        RCLCPP_ERROR(logger_, "PerformHomingProcedure: SetHomingSpeeds failed");
        return false;
    }
    
    if (!SetHomingStartingVelocity(homing_param_.starting_velocity)) {
        RCLCPP_ERROR(logger_, "PerformHomingProcedure: SetHomingStartingVelocity failed");
        return false;
    }
    
    // ホームオフセットの設定 実際はCANからは書き込めず、MEXE2でドライバに直接書き込む必要がある。
    if (!SetHomeOffset(homing_param_.offset)) {
        RCLCPP_ERROR(logger_, "PerformHomingProcedure: SetHomeOffset failed");
        return false;
    }
    
    // トルクリミットの設定 実際はCANからは書き込めず、MEXE2でドライバに直接書き込む必要がある。
    if (!SetHomingTruqueLimit(homing_param_.torque_limit)) {
        RCLCPP_ERROR(logger_, "PerformHomingProcedure: SetHomingTruqueLimit failed");
        return false;
    }
    
    if (!SetHomingBackwardsteps(homing_param_.homePosition)) {
        RCLCPP_ERROR(logger_, "PerformHomingProcedure: SetHomingBackwardsteps failed");
        return false;
    }
    if (!SetModesOfOperation(OperationMode::kHomingMode)) {
        RCLCPP_ERROR(logger_, "PerformHomingProcedure: SetModesOfOperation failed");
        return false;
    }
    
    if (!SetHomingMethod(homing_param_.method)) {
            RCLCPP_ERROR(logger_, "PerformHomingProcedure: SetHomingMethod failed");
            return false;
    }
        
    if (!SetHomingMode(HomingMode::PushMotionMode)) {
            RCLCPP_ERROR(logger_, "PerformHomingProcedure: SetHomingMethod(PushMotionMode) failed");
            return false;
    }
    
    uint8_t mode = 0;
    if (!DisplayModesOfOperation(mode)) {
        RCLCPP_ERROR(logger_, "PerformHomingProcedure: SetHomingMethod(PushMotionMode) failed");
        return false;
    }    
    
    if (!StartMotorToSwitchedOn()) {
            RCLCPP_ERROR(logger_, "PerformHomingProcedure: StartMotorToSwitchedOn failed");
            return false;
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    
    if (!EnableOperation()) {
            RCLCPP_ERROR(logger_, "PerformHomingProcedure: EnableOperation failed");
            return false;
    }
    
    if (!MonitorHomingDriveProfileOperationReady()) {
        RCLCPP_ERROR(logger_, "MonitorHomingDriveProfileOperationReady: Failed to get motor status during homing monitoring");
        return false;
    }
        
    // ホーミング動作の開始
    if (!StartHomingOperation()) {
            RCLCPP_ERROR(logger_, "PerformHomingProcedure: StartHomingOperation failed");
            return false;
    }
    
    if (!MonitorHomingProcedure()) {
            RCLCPP_ERROR(logger_, "PerformHomingProcedure: MonitorHomingProcedure failed");
            return false;
    }
    RCLCPP_INFO(logger_, "Homing procedure successfully started");
    return true;
}

bool OrientalMotorController::StartHomingOperation() {
    return SendControlWord(kStartHomingOperationValue);
  }

bool OrientalMotorController::SetHomingMethod(uint8_t homing_method) {
  static const uint8_t kSdoDownloadExpedited1Cmd = 0x2F;
    static const uint16_t kHomingMethodObject = 0x6098;
    static const uint8_t kHomingMethodSubindex = 0x00;
    uint16_t value = static_cast<uint16_t>(homing_method);
    std::vector<uint8_t> request_data = { 
        kSdoDownloadExpedited1Cmd,
        static_cast<uint8_t>(kHomingMethodObject & 0xFF),
        static_cast<uint8_t>((kHomingMethodObject >> 8) & 0xFF),
        kHomingMethodSubindex,
        static_cast<uint8_t>(value & 0xFF),
        static_cast<uint8_t>((value >> 8) & 0xFF),
        0x00, 0x00 
    };

    std::vector<uint8_t> response_data;
    if (!SdoTransaction(request_data, kSdoExpectedResponseDownload, response_data)) {
        return false;
    }
    RCLCPP_INFO(logger_, "Homing Method successfully set to: 0x%04X", homing_method);
    return true;
}

bool OrientalMotorController::SetHomingMode(HomingMode homing_mode) {
  static const uint8_t kSdoDownloadExpedited1Cmd = 0x2F;
  static const uint16_t kHomingModeObject = 0x4160;
  static const uint8_t kHomingModeSubindex = 0x00;
  uint16_t value = static_cast<uint16_t>(homing_mode);
  std::vector<uint8_t> request_data = { 
      kSdoDownloadExpedited1Cmd,
      static_cast<uint8_t>(kHomingModeObject & 0xFF),
      static_cast<uint8_t>((kHomingModeObject >> 8) & 0xFF),
      kHomingModeSubindex,
      static_cast<uint8_t>(value & 0xFF),
      static_cast<uint8_t>((value >> 8) & 0xFF),
      0x00, 0x00 
  };

  std::vector<uint8_t> response_data;
  if (!SdoTransaction(request_data, kSdoExpectedResponseDownload, response_data)) {
       return false;
  }
  RCLCPP_INFO(logger_, "Homing mode successfully set to: %u", value);
  return true;
}

bool OrientalMotorController::SetHomingSpeeds(uint32_t detection_speed, uint32_t zero_search_speed) {
    // Subindex 01: 検出速度の設定
    {
        static const uint8_t kSdoDownload4Cmd = 0x23;
        static const uint16_t kHomingSpeedsObject = 0x6099;
        const uint8_t subindex = 0x01;
        std::vector<uint8_t> request_data = {
            kSdoDownload4Cmd,
            static_cast<uint8_t>(kHomingSpeedsObject & 0xFF),
            static_cast<uint8_t>((kHomingSpeedsObject >> 8) & 0xFF),
            subindex,
            static_cast<uint8_t>(detection_speed & 0xFF),
            static_cast<uint8_t>((detection_speed >> 8) & 0xFF),
            static_cast<uint8_t>((detection_speed >> 16) & 0xFF),
            static_cast<uint8_t>((detection_speed >> 24) & 0xFF)
        };
        std::vector<uint8_t> response_data;
        if (!SdoTransaction(request_data, kSdoExpectedResponseDownload, response_data)) {
            return false;
        }
        RCLCPP_INFO(logger_, "Homing detection speed successfully set to: %u", detection_speed);
    }
    // Subindex 02: ゼロ検索速度の設定
    {
        static const uint8_t kSdoDownload4Cmd = 0x23;
        static const uint16_t kHomingSpeedsObject = 0x6099;
        const uint8_t subindex = 0x02;
        std::vector<uint8_t> request_data = {
            kSdoDownload4Cmd,
            static_cast<uint8_t>(kHomingSpeedsObject & 0xFF),
            static_cast<uint8_t>((kHomingSpeedsObject >> 8) & 0xFF),
            subindex,
            static_cast<uint8_t>(zero_search_speed & 0xFF),
            static_cast<uint8_t>((zero_search_speed >> 8) & 0xFF),
            static_cast<uint8_t>((zero_search_speed >> 16) & 0xFF),
            static_cast<uint8_t>((zero_search_speed >> 24) & 0xFF)
        };
        std::vector<uint8_t> response_data;
        if (!SdoTransaction(request_data, kSdoExpectedResponseDownload, response_data)) {
            return false;
        }
        RCLCPP_INFO(logger_, "Homing zero search speed successfully set to: %u", zero_search_speed);
    }
    return true;
}

bool OrientalMotorController::SetHomingAcceleration(uint32_t acceleration) {
    static const uint8_t kSdoDownload4Cmd = 0x23;
    static const uint16_t kHomingAccelObject = 0x609A;
    static const uint8_t kHomingAccelSubindex = 0x00;
    std::vector<uint8_t> request_data = {
        kSdoDownload4Cmd,
        static_cast<uint8_t>(kHomingAccelObject & 0xFF),
        static_cast<uint8_t>((kHomingAccelObject >> 8) & 0xFF),
        kHomingAccelSubindex,
        static_cast<uint8_t>(acceleration & 0xFF),
        static_cast<uint8_t>((acceleration >> 8) & 0xFF),
        static_cast<uint8_t>((acceleration >> 16) & 0xFF),
        static_cast<uint8_t>((acceleration >> 24) & 0xFF)
    };
    std::vector<uint8_t> response_data;
    if (!SdoTransaction(request_data, kSdoExpectedResponseDownload, response_data)) {
        return false;
    }
    RCLCPP_INFO(logger_, "Homing Acceleration successfully set to: %u", acceleration);
    return true;
}

bool OrientalMotorController::SetHomeOffset(int32_t offset) {
    static const uint8_t kSdoDownload4Cmd = 0x23;
    static const uint16_t kHomeOffsetObject = 0x607C;
    static const uint8_t kHomeOffsetSubindex = 0x00;
    std::vector<uint8_t> request_data = {
        kSdoDownload4Cmd,
        static_cast<uint8_t>(kHomeOffsetObject & 0xFF),
        static_cast<uint8_t>((kHomeOffsetObject >> 8) & 0xFF),
        kHomeOffsetSubindex,
        static_cast<uint8_t>(offset & 0xFF),
        static_cast<uint8_t>((offset >> 8) & 0xFF),
        static_cast<uint8_t>((offset >> 16) & 0xFF),
        static_cast<uint8_t>((offset >> 24) & 0xFF)
    };
    std::vector<uint8_t> response_data;
    if (!SdoTransaction(request_data, kSdoExpectedResponseDownload, response_data)) {
        return false;
    }
    RCLCPP_INFO(logger_, "Home Offset successfully set to: %d", offset);
    return true;
}

bool OrientalMotorController::SetModesOfOperation(OperationMode mode) {
    static const uint8_t kSdoDownloadExpedited1Cmd = 0x2F;
    static const uint16_t kModesOfOperationObject = 0x6060;
    static const uint8_t kModesOfOperationSubindex = 0x00;
    std::vector<uint8_t> request_data = {
        kSdoDownloadExpedited1Cmd,
        static_cast<uint8_t>(kModesOfOperationObject & 0xFF),
        static_cast<uint8_t>((kModesOfOperationObject >> 8) & 0xFF),
        kModesOfOperationSubindex,
        static_cast<uint8_t>(mode),
        0x00, 0x00, 0x00
    };
    std::vector<uint8_t> response_data;
    if (!SdoTransaction(request_data, kSdoExpectedResponseDownload, response_data)) {
        return false;
    }
    RCLCPP_INFO(logger_, "Modes of Operation successfully set to: %u", static_cast<unsigned int>(mode));
    return true;
}

bool OrientalMotorController::MonitorHomingProcedure() {
    uint16_t curr_status = 0;
    int interruptCount = 0;
    int attainedInProgressCount = 0;
    while (true) {
         if (!GetMotorStatus(&curr_status)) {
             RCLCPP_ERROR(logger_, "MonitorHomingProcedure: Failed to get motor status during homing monitoring");
             return false;
         }
         // 7bitが1ならば Alarm occurred として異常終了
         if (curr_status & (1 << 6)) {
             RCLCPP_ERROR(logger_, "Alarm occurred");
             return false;
         }
         int b13 = (curr_status >> 13) & 1;
         int b12 = (curr_status >> 12) & 1;
         int b10 = (curr_status >> 10) & 1;
         int key = (b13 << 2) | (b12 << 1) | b10;
         switch (key) {
             case 0:  // 0,0,0
                 RCLCPP_INFO(logger_, "Homing procedure is in progress");
                 interruptCount = 0;
                 attainedInProgressCount = 0;
                 break;
             case 1:  // 0,0,1
                 RCLCPP_WARN(logger_, "Homing procedure was interrupted or has not yet started");
                 interruptCount++;
                 if (interruptCount >= 5) {
                     RCLCPP_ERROR(logger_, "Homing procedure abnormal: interrupted/not started 5 times consecutively");
                     return false;
                 }
                 break;
             case 2:  // 0,1,0
                 RCLCPP_WARN(logger_, "Homing is attained, but the operation is still in progress");
                 attainedInProgressCount++;
                 if (attainedInProgressCount >= 5) {
                     RCLCPP_ERROR(logger_, "Homing procedure abnormal: attained but in progress 5 times consecutively");
                     return false;
                 }
                 break;
             case 3:  // 0,1,1
                 RCLCPP_INFO(logger_, "Homing procedure was completed successfully");
                 return true;
             case 4:  // 1,0,0
                 RCLCPP_ERROR(logger_, "A homing error occurred and the velocity is not 0");
                 return false;
             case 5:  // 1,1,0
                 RCLCPP_ERROR(logger_, "A homing error occurred and the velocity is 0");
                 return false;
             default:
                 RCLCPP_ERROR(logger_, "Unknown homing state");
                 return false;
         }
         std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

bool OrientalMotorController::StartMotorToSwitchedOn() {
    if (!FaultReset()) {
         RCLCPP_ERROR(logger_, "StartMotorToSwitchedOn: FaultReset failed");
         return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    
    if (!Shutdown()) {
         RCLCPP_ERROR(logger_, "StartMotorToSwitchedOn: Shutdown failed");
         return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    
    if (!SwitchOn()) {
         RCLCPP_ERROR(logger_, "StartMotorToSwitchedOn: SwitchOn failed");
         return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    RCLCPP_INFO(logger_, "Motor successfully switched on");
    return true;
}

bool OrientalMotorController::SetHomingStartingVelocity(uint32_t starting_velocity) {
    static const uint8_t kSdoDownload4Cmd = 0x23;
    static const uint16_t kStartingVelocityObject = 0x4163;
    static const uint8_t kStartingVelocitySubindex = 0x00;
    std::vector<uint8_t> request_data = {
        kSdoDownload4Cmd,
        static_cast<uint8_t>(kStartingVelocityObject & 0xFF),
        static_cast<uint8_t>((kStartingVelocityObject >> 8) & 0xFF),
        kStartingVelocitySubindex,
        static_cast<uint8_t>(starting_velocity & 0xFF),
        static_cast<uint8_t>((starting_velocity >> 8) & 0xFF),
        static_cast<uint8_t>((starting_velocity >> 16) & 0xFF),
        static_cast<uint8_t>((starting_velocity >> 24) & 0xFF)
    };
    std::vector<uint8_t> response_data;
    if (!SdoTransaction(request_data, kSdoExpectedResponseDownload, response_data)) {
         return false;
    }
    RCLCPP_INFO(logger_, "Starting velocity successfully set to: %u", starting_velocity);
    return true;
}

bool OrientalMotorController::SetHomingTruqueLimit(uint16_t torque_limit) {
    static const uint8_t kSdoDownloadCmd = 0x2B;
    static const uint16_t kHomingTruqueLimitObject = 0x415F; // SDOオブジェクトID (仮定)
    static const uint8_t kHomingTruqueLimitSubindex = 0x00;
    std::vector<uint8_t> request_data = {
        kSdoDownloadCmd,
        static_cast<uint8_t>(kHomingTruqueLimitObject & 0xFF),
        static_cast<uint8_t>((kHomingTruqueLimitObject >> 8) & 0xFF),
        kHomingTruqueLimitSubindex,
        static_cast<uint8_t>(torque_limit & 0xFF),
        static_cast<uint8_t>((torque_limit >> 8) & 0xFF),
        0x00, 0x00
    };
    std::vector<uint8_t> response_data;
    if (!SdoTransaction(request_data, kSdoExpectedResponseDownload, response_data)) {
         RCLCPP_ERROR(logger_, "SetHomingTruqueLimit: Failed to set homing torque limit to: 0x%04X", torque_limit);
         return false;
    }
    RCLCPP_INFO(logger_, "SetHomingTruqueLimit: Homing torque limit set to: 0x%04X", torque_limit);
    return true;
}

bool OrientalMotorController::SetHomingBackwardsteps(uint32_t backward_steps) {
    static const uint8_t kSdoDownload4Cmd = 0x23;
    static const uint16_t kBackwardStepsObject = 0x4169; // Placeholder object ID for backward steps
    static const uint8_t kBackwardStepsSubindex = 0x00;
    std::vector<uint8_t> request_data = {
         kSdoDownload4Cmd,
         static_cast<uint8_t>(kBackwardStepsObject & 0xFF),
         static_cast<uint8_t>((kBackwardStepsObject >> 8) & 0xFF),
         kBackwardStepsSubindex,
         static_cast<uint8_t>(backward_steps & 0xFF),
         static_cast<uint8_t>((backward_steps >> 8) & 0xFF),
         static_cast<uint8_t>((backward_steps >> 16) & 0xFF),
         static_cast<uint8_t>((backward_steps >> 24) & 0xFF)
    };
    std::vector<uint8_t> response_data;
    if (!SdoTransaction(request_data, kSdoExpectedResponseDownload, response_data)) {
         RCLCPP_ERROR(logger_, "SetHomingBackwardsteps: Failed to set backward steps to: %u", backward_steps);
         return false;
    }
    RCLCPP_INFO(logger_, "SetHomingBackwardsteps: Backward steps set to: %u", backward_steps);
    return true;
}

bool OrientalMotorController::DisplayModesOfOperation(uint8_t &mode) {
    static const uint8_t kSdoUploadRequestCmd = 0x40;
    static const uint8_t kSdoExpectedResponseUpload1 = 0x4F;
    // Using the same object for Modes of Operation as in SetModesOfOperation
    static const uint16_t kModesOfOperationObject = 0x6060;
    static const uint8_t kModesOfOperationSubindex = 0x00;
    std::vector<uint8_t> request_data = {
        kSdoUploadRequestCmd,
        static_cast<uint8_t>(kModesOfOperationObject & 0xFF),
        static_cast<uint8_t>((kModesOfOperationObject >> 8) & 0xFF),
        kModesOfOperationSubindex,
        0x00, 0x00, 0x00, 0x00
    };
    std::vector<uint8_t> response_data;
    if (!SdoTransaction(request_data, kSdoExpectedResponseUpload1, response_data)) {
         RCLCPP_ERROR(logger_, "DisplayModesOfOperation: Failed to read current mode");
         return false;
    }
    uint8_t mode_value = response_data[4];
    mode = mode_value;
    switch (mode_value) {
        case 1:
            RCLCPP_INFO(logger_, "DisplayModes :Profile Position Mode");
            break;
        case 3:
            RCLCPP_INFO(logger_, "DisplayModes :Profile Velocity Mode");
            break;
        case 4:
            RCLCPP_INFO(logger_, "DisplayModes :Profile Torque Mode");
            break;
        case 6:
            RCLCPP_INFO(logger_, "DisplayModes :Homing Mode");
            break;
        default:
            RCLCPP_WARN(logger_, "DisplayModes :Unknown Modes of Operation: %u", mode_value);
            break;
    }
    return true;
}

bool OrientalMotorController::MonitorHomingDriveProfileOperationReady() {
    const int maxIterations = 50; // 50 iterations x 20ms = 1 second
    uint16_t status = 0;
    for (int i = 0; i < maxIterations; i++) {
        if (!GetMotorStatus(&status)) {
            RCLCPP_ERROR(logger_, "MonitorHomingDriveProfileOperationReady: GetMotorStatus failed");
            return false;
        }
        // Assuming the 'Drive profile operation ready' flag is represented by bit 0 being 1.
        if (status & 0x0001) {
            RCLCPP_INFO(logger_, "Drive profile operation ready detected");
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    RCLCPP_ERROR(logger_, "MonitorHomingDriveProfileOperationReady: Drive profile operation ready not detected within 1 second");
    return false;
}

}
