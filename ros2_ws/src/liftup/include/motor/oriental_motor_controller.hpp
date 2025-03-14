#ifndef ORIENTAL_MOTOR_CONTROLLER_HPP_
#define ORIENTAL_MOTOR_CONTROLLER_HPP_

#include "motor_controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "can/can_interface.hpp"
#include "can/socket_can_interface.hpp"
#include <string>  // std::string 利用のため

namespace motor_controller {

// CAN ID とインターフェイスネームを保持する構造体
struct CanParameters {
  uint32_t can_id;
  std::string interface_name;
};

enum class MotorStatus {
  kNotReadyToSwitchOn,
  kSwitchOnDisabled,
  kReadyToSwitchOn,
  kSwitchedOn,
  kOperationEnabled,
  kQuickStopActive,
  kFaultReactionActive,
  kFault
};

enum class OperationMode {
  kProfilePositionMode = 1,
  kProfileVelocityMode = 3,
  kProfileTorqueMode = 4,
  kHomingMode = 6
};

enum class HomingMode {
    TwoSensorMode,         // 2-sensor mode
    ThreeSensorMode,       // 3-sensor mode
    OneWayRotationMode,    // One-way rotation mode
    PushMotionMode         // Push-motion mode
};

// Homingパラメータを保持する構造体
struct HomingParameter {
  uint8_t method;
  uint32_t acceleration;
  int32_t offset;
  uint16_t torque_limit;
  uint32_t timeout;
  uint32_t speed_serch_for_switch;
  uint32_t speed_serch_for_zero;
  uint32_t starting_velocity;
  uint32_t homePosition;
};

// 新規構造体: 位置モード用パラメータを保持する構造体
struct PositionModeParameter {
  uint16_t torque_limit;
  int32_t software_position_limit_positive;
  int32_t software_position_limit_negative;
  uint32_t profile_velocity;
  uint32_t end_velocity;
  uint32_t profile_acceleration;
  uint32_t profile_deceleration;
  uint32_t quick_stop_deceleration;
};

class OrientalMotorController : public MotorControllerInterface {
 public:
  // コンストラクタに PositionModeParameter を追加
  explicit OrientalMotorController(const CanParameters &params, const HomingParameter &homing_param, const PositionModeParameter &pos_mode_param);
  ~OrientalMotorController() override;

  bool GetMotorStatus(uint16_t *out_status) override;
  bool GetTorqueActualValue(uint16_t *out_torque) override;
  bool GetVelocityActualValue(uint32_t *out_velocity) override;
  bool GetPositionActualValue(uint32_t *out_position) override;
  bool SetMaxTorque(uint16_t max_torque) override;
  
  MotorStatus GetInternalStatus() const {return status_;}
  void SetCanId(uint32_t can_id);
  void SetHomingParameter(const HomingParameter &homing_param) {
    homing_param_ = homing_param;
  }
  
  // 新規メソッド: Controlwordを利用してホーミング動作を開始する
  bool PerformHomingProcedure();  
  bool StartMotorToSwitchedOn();  

  bool EnableOperation();
  bool DisableOperation();

  // 新規コード: 位置モードパラメータを設定するメソッド
  bool SetSoftwarePositionLimits(int32_t negative_limit, int32_t positive_limit);
  bool SetProfileVelocity(uint32_t velocity);
  bool SetEndVelocity(uint32_t end_velocity);
  bool SetProfileAcceleration(uint32_t acceleration);
  bool SetProfileDeceleration(uint32_t deceleration);
  bool SetQuickStopDeceleration(uint32_t quick_stop_deceleration);  
  bool MonitorPositionDriveProfileOperationReady();
  
 private:
  rclcpp::Logger logger_;
  std::shared_ptr<can_control::CanInterface> can_interface_;
  uint32_t can_id_;
  MotorStatus status_;
  HomingParameter homing_param_;   
  PositionModeParameter pos_mode_param_;  // 新規メンバ

  bool UpdateMotorState();
  void HandleMotorFault();

  //commonメソッド
  bool FaultReset();
  bool Shutdown();
  bool SwitchOn();  
  bool DisableVoltage(); 

  bool SetModesOfOperation(OperationMode mode);
  bool SendControlWord(uint16_t control_value);
  bool SdoTransaction(const std::vector<uint8_t> &request_data,
    uint8_t expected_response_cmd,
    std::vector<uint8_t> &response_data);

  //Homing関連メソッド
  //いくつかのパラメータはCANから設定することができない。(oriental motorの不具合っぽい)
  //そのため、MEXE02で設定する必要がある。
  //Homing_method , Homing_TruqueLimit , HomeOffsetなど
  //回転方向もCANから設定できないため、MEXE02で設定する。
  bool SetHomingMode(HomingMode homing_mode);
  bool SetHomingMethod(uint8_t homing_method);
  bool SetHomingSpeeds(uint32_t detection_speed, uint32_t zero_search_speed);
  bool SetHomingAcceleration(uint32_t acceleration);
  bool SetHomeOffset(int32_t offset);
  bool StartHomingOperation();
  bool SetHomingStartingVelocity(uint32_t starting_velocity);
  bool SetHomingTruqueLimit(uint16_t torque_limit);
  bool SetHomingBackwardsteps(uint32_t backward_steps);
  bool MonitorHomingProcedure();
  bool MonitorHomingDriveProfileOperationReady();


  bool DisplayModesOfOperation(uint8_t &mode);
  


  

  void LogCanParameters(const CanParameters &params);     // デバッグ用: CanParametersをログ出力
  void LogHomingParameters(const HomingParameter &hp);      // デバッグ用: HomingParameterをログ出力
};

}  // namespace motor_controller

#endif  // ORIENTAL_MOTOR_CONTROLLER_HPP_