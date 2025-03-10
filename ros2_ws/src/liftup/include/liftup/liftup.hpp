#ifndef LIFTUP_HPP_
#define LIFTUP_HPP_

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "motor/motor_controller_interface.hpp"
#include "motor/oriental_motor_controller.hpp"

class Liftup {
public:
  Liftup(uint32_t can_id);
  ~Liftup();

  void Start();
  void Stop();
  std::string GetStatus() const;

private:
  bool is_running_;
  rclcpp::Logger logger_;
  std::shared_ptr<motor_controller::OrientalMotorController> motor_controller_;
};

#endif  // LIFTUP_HPP_