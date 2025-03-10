#include "liftup/liftup_node.hpp"

LiftupNode::LiftupNode() : Node("liftup_node") {
  RCLCPP_INFO(this->get_logger(), "LiftupNode started");

  // パラメータの宣言と初期値の設定
  this->declare_parameter<int>("can_id", 0x123);

  // パラメータからCAN IDを取得
  int can_id;
  this->get_parameter("can_id", can_id);

  // Liftupクラスの初期化
  liftup_ = std::make_shared<Liftup>(static_cast<uint32_t>(can_id));

  // ノード初期化処理（パラメータ宣言、パブリッシャー、サブスクリプション、サービスなど）を追加してください。

}

LiftupNode::~LiftupNode() {
  // 必要に応じたクリーンアップ処理を記述してください。
}
