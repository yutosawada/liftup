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

  // モータのステータスを取得する
  virtual bool GetMotorStatus(uint16_t *out_status) = 0;
  virtual bool GetTorqueActualValue(uint16_t *out_torque) = 0;
  virtual bool GetVelocityActualValue(uint32_t *out_velocity) = 0;
  virtual bool GetPositionActualValue(uint32_t *out_position) = 0;

  virtual bool SetMaxTorque(uint16_t max_torque) = 0;
};




}  // namespace motor_controller

#endif  // MOTOR_CONTROLLER_INTERFACE_HPP_