#ifndef ORIENTAL_MOTOR_CONSTANTS_HPP_
#define ORIENTAL_MOTOR_CONSTANTS_HPP_

#include "motor_controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "can/can_interface.hpp"
#include "can/socket_can_interface.hpp"
#include <string>  // std::string 利用のため

namespace motor_controller {

constexpr uint32_t kSdoRequestBaseId         = 0x600;
constexpr uint32_t kSdoResponseBaseId        = 0x580;
constexpr size_t   kSdoDlc                   = 8;
constexpr int      kReceiveTimeoutMs         = 1000;

constexpr uint8_t  kSdoExpectedResponseUpload   = 0x4B;
constexpr uint8_t  kSdoExpectedResponseDownload = 0x60;

constexpr uint16_t kFaultResetValue          = 0x0080;
constexpr uint16_t kShutdownValue            = 0x0006;
constexpr uint16_t kSwitchOnValue            = 0x0007;
constexpr uint16_t kEnableOperationValue     = 0x000F;
constexpr uint16_t kDisableVoltageValue      = 0x0000;
constexpr uint16_t kDisableOperationValue    = 0x0007;
constexpr uint16_t kStartHomingOperationValue= 0x001F;

}  

#endif  // ORIENTAL_MOTOR_CONTROLLER_HPP_