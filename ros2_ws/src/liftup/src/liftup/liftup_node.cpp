#include "liftup/liftup_node.hpp"
#include "liftup/liftup.hpp"
#include "motor/oriental_motor_controller.hpp"

LiftupNode::LiftupNode() : Node("liftup_node") {
  RCLCPP_INFO(this->get_logger(), "LiftupNode started");

  // パラメータの宣言と初期値の設定（canparameters 配下に変更）
  this->declare_parameter<int>("canparameters.can_id", 0x999);
  this->declare_parameter<std::string>("canparameters.interface_name", "default_interface");

  // パラメータからCAN IDとインターフェイスネームを取得
  motor_controller::CanParameters params;
  this->get_parameter("canparameters.can_id", params.can_id);
  this->get_parameter("canparameters.interface_name", params.interface_name);

  // ホーミングパラメータの宣言と取得
  this->declare_parameter<int>("homing_parameter.method", 8);
  this->declare_parameter<int>("homing_parameter.acceleration", 100);
  this->declare_parameter<int>("homing_parameter.offset", 0);
  this->declare_parameter<int>("homing_parameter.torque_limit", 100);
  this->declare_parameter<int>("homing_parameter.timeout", 1000);
  this->declare_parameter<int>("homing_parameter.homePosition", 1000);
  this->declare_parameter<int>("homing_parameter.speed_serch_for_switch", 5);
  this->declare_parameter<int>("homing_parameter.speed_serch_for_zero", 5);
  this->declare_parameter<int>("homing_parameter.starting_velocity", 5);

  motor_controller::HomingParameter homing_param;
  this->get_parameter("homing_parameter.method", homing_param.method);
  this->get_parameter("homing_parameter.acceleration", homing_param.acceleration);
  this->get_parameter("homing_parameter.offset", homing_param.offset);
  this->get_parameter("homing_parameter.torque_limit", homing_param.torque_limit);
  this->get_parameter("homing_parameter.timeout", homing_param.timeout);
  this->get_parameter("homing_parameter.homePosition", homing_param.homePosition);
  this->get_parameter("homing_parameter.speed_serch_for_switch", homing_param.speed_serch_for_switch);
  this->get_parameter("homing_parameter.speed_serch_for_zero", homing_param.speed_serch_for_zero);
  this->get_parameter("homing_parameter.starting_velocity", homing_param.starting_velocity);

  // 新規: position_mode_parameterの宣言
  this->declare_parameter<int16_t>("position_mode_parameter.torque_limit", 100);
  this->declare_parameter<int32_t>("position_mode_parameter.software_position_limit_positive", 10000);
  this->declare_parameter<int32_t>("position_mode_parameter.software_position_limit_negative", -10000);
  this->declare_parameter<int32_t>("position_mode_parameter.profile_velocity", 300);
  this->declare_parameter<int32_t>("position_mode_parameter.end_velocity", 0);
  this->declare_parameter<int32_t>("position_mode_parameter.profile_acceleration", 1000);
  this->declare_parameter<int32_t>("position_mode_parameter.profile_deceleration", 1000);
  this->declare_parameter<int32_t>("position_mode_parameter.quick_stop_deceleration", 1500);

  motor_controller::PositionModeParameter pos_mode_param;
  this->get_parameter("position_mode_parameter.torque_limit", pos_mode_param.torque_limit);
  this->get_parameter("position_mode_parameter.software_position_limit_positive", pos_mode_param.software_position_limit_positive);
  this->get_parameter("position_mode_parameter.software_position_limit_negative", pos_mode_param.software_position_limit_negative);
  this->get_parameter("position_mode_parameter.profile_velocity", pos_mode_param.profile_velocity);
  this->get_parameter("position_mode_parameter.end_velocity", pos_mode_param.end_velocity);
  this->get_parameter("position_mode_parameter.profile_acceleration", pos_mode_param.profile_acceleration);
  this->get_parameter("position_mode_parameter.profile_deceleration", pos_mode_param.profile_deceleration);
  this->get_parameter("position_mode_parameter.quick_stop_deceleration", pos_mode_param.quick_stop_deceleration);

  // Liftupクラスの初期化（ホーミングパラメータと位置モードパラメータを渡す）
  liftup_ = std::make_shared<Liftup>(params, homing_param, pos_mode_param);

  // サービスの初期化
  get_motor_status_service_ = this->create_service<liftup::srv::GetMotorStatus>(
    "get_motor_status", std::bind(&LiftupNode::HandleGetMotorStatus, this, std::placeholders::_1, std::placeholders::_2));

  // MotorOn サービスの初期化
  motor_on_service_ = this->create_service<liftup::srv::MotorOn>(
    "motor_on", std::bind(&LiftupNode::HandleMotorOn, this, std::placeholders::_1, std::placeholders::_2));

  motor_off_service_ = this->create_service<liftup::srv::MotorOff>(
    "motor_off", std::bind(&LiftupNode::HandleMotorOff, this, std::placeholders::_1, std::placeholders::_2));
}

LiftupNode::~LiftupNode() {
  // 必要に応じたクリーンアップ処理を記述してください。
}

void LiftupNode::HandleGetMotorStatus(
  const std::shared_ptr<liftup::srv::GetMotorStatus::Request> request,
  std::shared_ptr<liftup::srv::GetMotorStatus::Response> response) {
  (void)request;  // リクエストは空
  uint16_t status;
  if (liftup_->GetMotorStatus(&status)) {
    response->status_value = status;
    // DS402のStatusword（例）の各ビットの意味（以下は一例）：
    response->ready_to_switch_on         = (status & (1 << 0)) != 0;
    response->switched_on                  = (status & (1 << 1)) != 0;
    response->operation_enabled            = (status & (1 << 2)) != 0;
    response->fault                        = (status & (1 << 3)) != 0;
    response->voltage_enabled              = (status & (1 << 4)) != 0;
    response->quick_stop                   = (status & (1 << 5)) != 0;
    response->switch_on_disabled           = (status & (1 << 6)) != 0;
    response->warning                      = (status & (1 << 7)) != 0;
    response->drive_profile_operation_ready= (status & (1 << 8)) != 0;
    response->remote                       = (status & (1 << 9)) != 0;
    response->target_reached               = (status & (1 << 10)) != 0;
    response->internal_limit_active        = (status & (1 << 11)) != 0;

  RCLCPP_INFO(get_logger(), "Retrieved Motor Status: 0x%04X", status);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to get motor status");
  }
}

void LiftupNode::HandleMotorOn(
  const std::shared_ptr<liftup::srv::MotorOn::Request> request,
  std::shared_ptr<liftup::srv::MotorOn::Response> response)
{
  (void)request;  // リクエストは空
  // Liftup クラスに追加した public メソッド MotorOn() を使用
  if (liftup_->MotorOn()) {
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "MotorOn command executed successfully.");
  } else {
    response->success = false;
    RCLCPP_ERROR(this->get_logger(), "Failed to execute MotorOn command.");
  }
}

void LiftupNode::HandleMotorOff(
  const std::shared_ptr<liftup::srv::MotorOff::Request> request,
  std::shared_ptr<liftup::srv::MotorOff::Response> response)
{
  (void)request;
  if (liftup_->MotorOff()) {
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "MotorOff command executed successfully.");
  } else {
    response->success = false;
    RCLCPP_ERROR(this->get_logger(), "Failed to execute MotorOff command.");
  }
}
