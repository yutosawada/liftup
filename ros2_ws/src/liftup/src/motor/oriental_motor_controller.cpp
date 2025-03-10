#include "motor/oriental_motor_controller.hpp"

namespace motor_controller {

OrientalMotorController::OrientalMotorController()
  : logger_(rclcpp::get_logger("OrientalMotorController")), can_id_(0x123) {  // デフォルトのCAN IDを設定
  RCLCPP_INFO(logger_, "OrientalMotorController initialized");

  // ソケットCANインターフェースの初期化
  can_interface_ = std::make_shared<can_control::SocketCanInterface>("can0");
}

OrientalMotorController::~OrientalMotorController() {
  RCLCPP_INFO(logger_, "OrientalMotorController destroyed");
}

void OrientalMotorController::SetCanId(uint32_t can_id) {
  
  can_id_ = can_id;
  RCLCPP_INFO(logger_, "CAN ID set to: 0x%X", can_id_);
}

bool OrientalMotorController::SendMotorCommand(const MotorCommand &command) {
  can_control::CanFrame frame;
  frame.arbitration_id = can_id_;  // 設定されたCAN IDを使用
  frame.dlc = 1;                   // データ長
  frame.data.push_back(command.target_velocity);  // 例としてターゲット速度を設定

  if (!can_interface_->Send(frame)) {
    RCLCPP_ERROR(logger_, "Failed to send CAN frame");
    return false;
  }

  RCLCPP_INFO(logger_, "Sent motor command with target velocity: %u", command.target_velocity);
  return true;
}



}  // namespace motor_controller