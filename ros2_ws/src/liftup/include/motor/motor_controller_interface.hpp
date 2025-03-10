#ifndef MOTOR_CONTROLLER_INTERFACE_HPP_
#define MOTOR_CONTROLLER_INTERFACE_HPP_

#include <cstdint>
#include <vector>

namespace motor_controller {

// モータ制御コマンドの構造体例（ここではターゲット速度のみ）
struct MotorCommand {
  uint32_t target_velocity;  // 単位: rpm（Target Velocity）
};

// 抽象インターフェースクラス
class MotorControllerInterface {
 public:
  virtual ~MotorControllerInterface() = default;

  // モータコントロールコマンドを送信する
  virtual bool SendMotorCommand(const MotorCommand &command) = 0;


};

}  // namespace motor_controller

#endif  // MOTOR_CONTROLLER_INTERFACE_HPP_