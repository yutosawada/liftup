#ifndef LIFTUP_NODE_HPP_
#define LIFTUP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "liftup/liftup.hpp"
#include "motor/motor_controller_interface.hpp"
#include "motor/oriental_motor_controller.hpp"

class LiftupNode : public rclcpp::Node {
public:
  LiftupNode();
  ~LiftupNode();

private:
  std::shared_ptr<Liftup> liftup_;
};

#endif  // LIFTUP_NODE_HPP_
