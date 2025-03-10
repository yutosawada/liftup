#ifndef ORIENTAL_MOTOR_CONTROLLER_HPP_
#define ORIENTAL_MOTOR_CONTROLLER_HPP_

#include "motor_controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "can/can_interface.hpp"
#include "can/socket_can_interface.hpp"

namespace motor_controller {

class OrientalMotorController : public MotorControllerInterface {
 public:
  OrientalMotorController();
  ~OrientalMotorController() override;

  bool SendMotorCommand(const MotorCommand &command) override;

  void SetCanId(uint32_t can_id);

 private:
  rclcpp::Logger logger_;
  std::shared_ptr<can_control::CanInterface> can_interface_;
  uint32_t can_id_;
};

}  // namespace motor_controller

#endif  // ORIENTAL_MOTOR_CONTROLLER_HPP_