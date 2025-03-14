#ifndef LIFTUP_HPP_
#define LIFTUP_HPP_

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "motor/motor_controller_interface.hpp"
#include "motor/oriental_motor_controller.hpp"

class Liftup {
public:
  Liftup(motor_controller::CanParameters params, motor_controller::HomingParameter homing_param, motor_controller::PositionModeParameter pos_mode_param);
  ~Liftup();

  bool MotorOn();
  bool MotorOff();
  bool ErrorReset();
  bool LiftUp();
  bool LiftDown();
  bool MoveHomePosition();

  bool SelectMode();

  bool GetMotorStatus(uint16_t *out_status) const;

  // 追加: モーターコントローラーへのゲッター
  std::shared_ptr<motor_controller::OrientalMotorController> GetMotorController() const {
    return motor_controller_;
  }

private:
  bool is_running_;
  rclcpp::Logger logger_;
  std::shared_ptr<motor_controller::OrientalMotorController> motor_controller_;
};

#endif  // LIFTUP_HPP_