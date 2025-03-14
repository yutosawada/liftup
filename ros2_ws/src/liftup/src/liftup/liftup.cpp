#include "liftup/liftup.hpp"
#include "motor/oriental_motor_controller.hpp"

Liftup::Liftup(motor_controller::CanParameters params, motor_controller::HomingParameter homing_param, motor_controller::PositionModeParameter pos_mode_param)
  : is_running_(false), 
    logger_(rclcpp::get_logger("Liftup")){
  RCLCPP_INFO(logger_, "Liftup initialized");
  motor_controller_ = std::make_shared<motor_controller::OrientalMotorController>(params, homing_param, pos_mode_param);

  //std::this_thread::sleep_for(std::chrono::seconds(5));
  //motor_controller_->EnableOperation();

/*
  // モーターを起動してSwitched On状態にする
  if (!motor_controller_->StartMotorToSwitchedOn()) {
    RCLCPP_ERROR(logger_, "PerformHomingProcedure: StartMotorToSwitchedOn failed");
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  // モータをoperation enabled状態にする
  if (!motor_controller_->EnableOperation()) {
      RCLCPP_ERROR(logger_, "PerformHomingProcedure: EnableOperation failed");
  }
*/


  
  std::this_thread::sleep_for(std::chrono::seconds(1));
  //motor_controller_->PerformHomingProcedure();

  // モーターコントローラーの初期化
  //motor_controller_ = std::make_shared<motor_controller::OrientalMotorController>(params,homing_param);
  //motor_controller_->InitializeMotor();
}

Liftup::~Liftup() {
  RCLCPP_INFO(logger_, "Liftup destroyed");
}

bool Liftup::GetMotorStatus(uint16_t *out_status) const {
  if (motor_controller_->GetMotorStatus(out_status)) {
    RCLCPP_INFO(logger_, "Motor status: 0x%X", *out_status);
    return true;
  } else {
    RCLCPP_ERROR(logger_, "Failed to get motor status");
    return false;
  }
}

// 追加: MotorOn メソッド
bool Liftup::MotorOn() {
  return motor_controller_->EnableOperation();
}

// 追加: MotorOff メソッド
bool Liftup::MotorOff() {
  return motor_controller_->DisableOperation();
}