#include "liftup/liftup.hpp"

Liftup::Liftup(uint32_t can_id)
  : is_running_(false), logger_(rclcpp::get_logger("Liftup")) {
  RCLCPP_INFO(logger_, "Liftup initialized");

  // モーターコントローラーの初期化
  motor_controller_ = std::make_shared<motor_controller::OrientalMotorController>();
  motor_controller_->SetCanId(can_id);
}

Liftup::~Liftup() {
  RCLCPP_INFO(logger_, "Liftup destroyed");
}

void Liftup::Start() {
  is_running_ = true;
  RCLCPP_INFO(logger_, "Liftup started");
}

void Liftup::Stop() {
  is_running_ = false;
  RCLCPP_INFO(logger_, "Liftup stopped");
}

std::string Liftup::GetStatus() const {
  return is_running_ ? "Running" : "Stopped";
}