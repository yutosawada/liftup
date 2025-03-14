#include "motor/oriental_motor_controller.hpp"
#include "motor/oriental_motor_constants.hpp"  // 新規インクルード
#include <chrono>  // added for sleep
#include <thread>  // added for sleep

namespace motor_controller {

void OrientalMotorController::LogCanParameters(const CanParameters &params) {
    RCLCPP_INFO(logger_, "CanParameters - CAN ID: 0x%X, Interface: %s",
                params.can_id, params.interface_name.c_str());
}

void OrientalMotorController::LogHomingParameters(const HomingParameter &hp) {
    RCLCPP_INFO(logger_, "HomingParameter - Method: %d, Acceleration: %u, Offset: %d, Torque Limit: %u, Timeout: %u, SpeedForSwitch: %u, SpeedForZero: %u, StartingVelocity: %u",
                hp.method, hp.acceleration, hp.offset, hp.torque_limit, hp.timeout,
                hp.speed_serch_for_switch, hp.speed_serch_for_zero, hp.starting_velocity);
}

OrientalMotorController::OrientalMotorController(const CanParameters &params, const HomingParameter &homing_param, const PositionModeParameter &pos_mode_param)
  : logger_(rclcpp::get_logger("OrientalMotorController")),
    can_interface_(std::make_shared<can_control::SocketCanInterface>(params.interface_name)),
    can_id_(params.can_id),
    status_(motor_controller::MotorStatus::kNotReadyToSwitchOn),
    homing_param_(homing_param),
    pos_mode_param_(pos_mode_param) {
  RCLCPP_INFO(logger_, "OrientalMotorController initialized");
  LogCanParameters(params);
  LogHomingParameters(homing_param_);
  // ソケットCANインターフェースの初期化：引数の interface_name を使用

  // モーターを Switched On 状態にする
  //if (!StartMotorToSwitchedOn()) {
  //  RCLCPP_ERROR(logger_, "Failed to switch on motor during initialization");
  //}
}

OrientalMotorController::~OrientalMotorController() {
  bool disabled = false;
  for (int i = 0; i < 3; ++i) {
      if (DisableVoltage()) {
          disabled = true;
          break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (!disabled) {
      RCLCPP_ERROR(logger_, "Failed to disable voltage in destructor after 3 retries");
  }
  RCLCPP_INFO(logger_, "OrientalMotorController destroyed");
}

void OrientalMotorController::SetCanId(uint32_t can_id) {
  can_id_ = can_id;
  RCLCPP_INFO(logger_, "CAN ID set to: 0x%X", can_id_);
}

bool OrientalMotorController::UpdateMotorState() {
  uint16_t status = 0;
  if (!GetMotorStatus(&status)) {
    RCLCPP_ERROR(logger_, "UpdateMotorState: Failed to retrieve motor status");
    return false;
  }
  
  // 7ビット目が1の場合は、状態を kFault に設定
  if (status & (1 << 6)) {  // 7ビット目（0x40）のチェック
    status_ = MotorStatus::kFault;
    HandleMotorFault();
  } else {
    // 下位4ビットで状態判定する例（実際の仕様に合わせて変更してください）
    switch (status & 0x000F) {
      //case 0x0000:
      //  status_ = MotorStatus::kNotReadyToSwitchOn;
      //  break;
      case 0x0001:
        status_ = MotorStatus::kSwitchOnDisabled;
        break;
      case 0x0002:
        status_ = MotorStatus::kReadyToSwitchOn;
        break;
      case 0x0003:
        status_ = MotorStatus::kSwitchedOn;
        break;
      case 0x000F:
        status_ = MotorStatus::kOperationEnabled;
        break;
      case 0x0005:
        status_ = MotorStatus::kQuickStopActive;
        break;
      //case 0x0006:
      //  status_ = MotorStatus::kFaultReactionActive;
      //  break;
      default:
        RCLCPP_WARN(logger_, "Unknown motor status: 0x%04X", status);
        break;
    }
  }

  RCLCPP_INFO(logger_, "Internal motor state updated: 0x%04X", static_cast<uint16_t>(status_));
  return true;
}

void OrientalMotorController::HandleMotorFault() {
  // TODO: 障害発生時の処理を実装してください
  RCLCPP_WARN(logger_, "Motor fault detected. Executing fault handling procedure.");
}



// 共通処理：SDO要求・応答のやりとりを行う
// request_data : 送信データ
// expected_response_cmd : 期待するレスポンス先頭バイト
// response_data : 受信したレスポンスデータ
bool OrientalMotorController::SdoTransaction(const std::vector<uint8_t> &request_data,
                                               uint8_t expected_response_cmd,
                                               std::vector<uint8_t> &response_data) {
  can_control::CanFrame request;
  request.arbitration_id = kSdoRequestBaseId + can_id_;
  request.dlc = kSdoDlc;
  request.data = request_data;

  if (!can_interface_->Send(request)) {
    RCLCPP_ERROR(logger_, "Failed to send SDO command.");
    return false;
  }

  can_control::CanFrame response;
  if (!can_interface_->Receive(&response, kReceiveTimeoutMs)) {
    RCLCPP_ERROR(logger_, "Failed to receive SDO response.");
    return false;
  }
  if (response.arbitration_id != (kSdoResponseBaseId + can_id_)) {
    RCLCPP_ERROR(logger_, "Unexpected SDO response CAN ID: 0x%X", response.arbitration_id);
    return false;
  }
  if (response.data.size() < kSdoDlc) {
    RCLCPP_ERROR(logger_, "SDO response data length is insufficient.");
    return false;
  }
  if (response.data[0] != expected_response_cmd) {
    RCLCPP_ERROR(logger_, "SDO response error: 0x%X", response.data[0]);
    return false;
  }
  response_data = response.data;
  return true;
}


bool OrientalMotorController::GetMotorStatus(uint16_t *out_status) {
  if (out_status == nullptr) {
    RCLCPP_ERROR(logger_, "GetMotorStatus: out_status is nullptr.");
    return false;
  }
  static const uint8_t kSdoUploadRequestCmd = 0x40;     // アップロード要求コマンド
  static const uint16_t kStatuswordObject = 0x6041;      // Statuswordオブジェクト
  static const uint8_t kStatuswordSubindex = 0x00;
  // kExpectedResponseCmd -> kSdoExpectedResponseUpload

  std::vector<uint8_t> request_data = { kSdoUploadRequestCmd,
                                        static_cast<uint8_t>(kStatuswordObject & 0xFF),
                                        static_cast<uint8_t>((kStatuswordObject >> 8) & 0xFF),
                                        kStatuswordSubindex,
                                        0x00, 0x00, 0x00, 0x00 };
  std::vector<uint8_t> response_data;
  if (!SdoTransaction(request_data, kSdoExpectedResponseUpload, response_data)) {
    return false;
  }
  uint32_t raw = response_data[4] |
                 (response_data[5] << 8) |
                 (response_data[6] << 16) |
                 (response_data[7] << 24);
  *out_status = static_cast<uint16_t>(raw & 0xFFFF);
  RCLCPP_INFO(logger_, "Retrieved Statusword: 0x%04X", *out_status);
  return true;
}

bool OrientalMotorController::GetTorqueActualValue(uint16_t *out_torque) {
  if (out_torque == nullptr) {
    RCLCPP_ERROR(logger_, "GetTorqueActualValue: out_torque is nullptr.");
    return false;
  }
  static const uint8_t kSdoUploadRequestCmd = 0x40;
  static const uint16_t kTorqueActualObject = 0x6077;
  static const uint8_t kTorqueActualSubindex = 0x00;

  std::vector<uint8_t> request_data = { kSdoUploadRequestCmd,
                                        static_cast<uint8_t>(kTorqueActualObject & 0xFF),
                                        static_cast<uint8_t>((kTorqueActualObject >> 8) & 0xFF),
                                        kTorqueActualSubindex,
                                        0x00, 0x00, 0x00, 0x00 };
  std::vector<uint8_t> response_data;
  if (!SdoTransaction(request_data, kSdoExpectedResponseUpload, response_data)) {
    return false;
  }
  uint32_t raw = response_data[4] |
                 (response_data[5] << 8) |
                 (response_data[6] << 16) |
                 (response_data[7] << 24);
  *out_torque = static_cast<uint16_t>(raw & 0xFFFF);
  RCLCPP_INFO(logger_, "Retrieved Torque Actual Value: 0x%04X", *out_torque);
  return true;
}

bool OrientalMotorController::GetVelocityActualValue(uint32_t *out_velocity) {
  if (out_velocity == nullptr) {
    RCLCPP_ERROR(logger_, "GetVelocityActualValue: out_velocity is nullptr.");
    return false;
  }
  static const uint8_t kSdoUploadRequestCmd = 0x40;
  static const uint16_t kVelocityActualObject = 0x606C;
  static const uint8_t kVelocityActualSubindex = 0x00;

  std::vector<uint8_t> request_data = { kSdoUploadRequestCmd,
                                        static_cast<uint8_t>(kVelocityActualObject & 0xFF),
                                        static_cast<uint8_t>((kVelocityActualObject >> 8) & 0xFF),
                                        kVelocityActualSubindex,
                                        0x00, 0x00, 0x00, 0x00 };
  std::vector<uint8_t> response_data;
  if (!SdoTransaction(request_data, kSdoExpectedResponseUpload, response_data)) {
    return false;
  }
  uint32_t raw = response_data[4] |
                 (response_data[5] << 8) |
                 (response_data[6] << 16) |
                 (response_data[7] << 24);
  *out_velocity = raw;
  RCLCPP_INFO(logger_, "Retrieved Velocity Actual Value: %u", *out_velocity);
  return true;
}

bool OrientalMotorController::GetPositionActualValue(uint32_t *out_position) {
  if (out_position == nullptr) {
    RCLCPP_ERROR(logger_, "GetPositionActualValue: out_position is nullptr.");
    return false;
  }
  static const uint8_t kSdoUploadRequestCmd = 0x40;
  static const uint16_t kPositionActualObject = 0x6064;
  static const uint8_t kPositionActualSubindex = 0x00;

  std::vector<uint8_t> request_data = { kSdoUploadRequestCmd,
                                        static_cast<uint8_t>(kPositionActualObject & 0xFF),
                                        static_cast<uint8_t>((kPositionActualObject >> 8) & 0xFF),
                                        kPositionActualSubindex,
                                        0x00, 0x00, 0x00, 0x00 };
  std::vector<uint8_t> response_data;
  if (!SdoTransaction(request_data, kSdoExpectedResponseUpload, response_data)) {
    return false;
  }
  uint32_t raw = response_data[4] |
                 (response_data[5] << 8) |
                 (response_data[6] << 16) |
                 (response_data[7] << 24);
  *out_position = raw;
  RCLCPP_INFO(logger_, "Retrieved Position Actual Value: %u", *out_position);
  return true;
}

bool OrientalMotorController::SetMaxTorque(uint16_t max_torque) {
  static const uint8_t kSdoDownloadCmd = 0x2B;
  static const uint16_t kMaxTorqueObject = 0x6072;
  static const uint8_t kMaxTorqueSubindex = 0x00;
  // kExpectedResponseCmd -> kSdoExpectedResponseDownload

  std::vector<uint8_t> request_data = { kSdoDownloadCmd,
                                        static_cast<uint8_t>(kMaxTorqueObject & 0xFF),
                                        static_cast<uint8_t>((kMaxTorqueObject >> 8) & 0xFF),
                                        kMaxTorqueSubindex,
                                        static_cast<uint8_t>(max_torque & 0xFF),
                                        static_cast<uint8_t>((max_torque >> 8) & 0xFF),
                                        0x00, 0x00 };
  std::vector<uint8_t> response_data;
  if (!SdoTransaction(request_data, kSdoExpectedResponseDownload, response_data)) {
    return false;
  }
  RCLCPP_INFO(logger_, "Max Torque command successful: 0x%04X", max_torque);
  return true;
}

bool OrientalMotorController::FaultReset() {
    return SendControlWord(kFaultResetValue);
}

bool OrientalMotorController::Shutdown() {
    return SendControlWord(kShutdownValue);
}

bool OrientalMotorController::SwitchOn() {
    return SendControlWord(kSwitchOnValue);
}

bool OrientalMotorController::EnableOperation() {
    return SendControlWord(kEnableOperationValue);
}

bool OrientalMotorController::DisableVoltage() {
  return SendControlWord(kDisableVoltageValue);
}

bool OrientalMotorController::DisableOperation() {
  return SendControlWord(kDisableOperationValue);
}

bool OrientalMotorController::SendControlWord(uint16_t control_value) {
    static const uint8_t kSdoDownloadCmd = 0x2B;
    static const uint16_t kControlwordObject = 0x6040;
    static const uint8_t kControlwordSubindex = 0x00;
    std::vector<uint8_t> request_data = {
        kSdoDownloadCmd,
        static_cast<uint8_t>(kControlwordObject & 0xFF),
        static_cast<uint8_t>((kControlwordObject >> 8) & 0xFF),
        kControlwordSubindex,
        static_cast<uint8_t>(control_value & 0xFF),
        static_cast<uint8_t>((control_value >> 8) & 0xFF),
        0x00, 0x00
    };
    std::vector<uint8_t> response_data;
    if (!SdoTransaction(request_data, kSdoExpectedResponseDownload, response_data)) {
         RCLCPP_ERROR(logger_, "SendControlWord: Failed to send 0x%04X", control_value);
         return false;
    }
    RCLCPP_INFO(logger_, "SendControlWord: 0x%04X sent successfully.", control_value);
    return true;
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

}  // namespace motor_controller