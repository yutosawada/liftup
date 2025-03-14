#ifndef LIFTUP_NODE_HPP_
#define LIFTUP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "liftup/liftup.hpp"
#include "motor/motor_controller_interface.hpp"
#include "motor/oriental_motor_controller.hpp"
#include "liftup/srv/get_motor_status.hpp"
// サービス型のヘッダー（MotorOnサービスを定義している前提）
#include "liftup/srv/motor_on.hpp"
#include "liftup/srv/motor_off.hpp"

class LiftupNode : public rclcpp::Node {
public:
  LiftupNode();
  ~LiftupNode();

private:
  std::shared_ptr<Liftup> liftup_;
  
  // デバッグ用
  void HandleGetMotorStatus(
    const std::shared_ptr<liftup::srv::GetMotorStatus::Request> request,
    std::shared_ptr<liftup::srv::GetMotorStatus::Response> response);

  // MotorOnサービス コールバック
  void HandleMotorOn(
    const std::shared_ptr<liftup::srv::MotorOn::Request> request,
    std::shared_ptr<liftup::srv::MotorOn::Response> response);

  // MotorOffサービス コールバック
  void HandleMotorOff(
    const std::shared_ptr<liftup::srv::MotorOff::Request> request,
    std::shared_ptr<liftup::srv::MotorOff::Response> response);

  // サービスの保持
  rclcpp::Service<liftup::srv::GetMotorStatus>::SharedPtr get_motor_status_service_;
  rclcpp::Service<liftup::srv::MotorOn>::SharedPtr motor_on_service_;
  rclcpp::Service<liftup::srv::MotorOff>::SharedPtr motor_off_service_;
};

#endif  // LIFTUP_NODE_HPP_
