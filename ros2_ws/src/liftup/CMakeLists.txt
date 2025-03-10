cmake_minimum_required(VERSION 3.8)
project(liftup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# サービス用
find_package(rosidl_default_generators REQUIRED)

# C++ノード用
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

# カスタムサービスの生成
# rosidl_generate_interfaces(${PROJECT_NAME}
#  "msg/liftupStatus.msg"
#  "srv/ServoOn.srv"
#  DEPENDENCIES builtin_interfaces
# )

# 生成されたヘッダを使う場合に必要
ament_export_dependencies(rosidl_default_runtime)

# インクルードディレクトリを追加
include_directories(
  include
)

# 実行可能ファイルのターゲットを定義
add_executable(liftup_node_entrypoint
  src/liftup/liftup.cpp  
  src/liftup/liftup_node.cpp
  src/liftup/liftup_node_entrypoint.cpp
  src/motor/oriental_motor_controller.cpp
  src/can/socket_can_interface.cpp  # ここに追加
)

# 必要な依存関係をリンク
ament_target_dependencies(liftup_node_entrypoint
  rclcpp
  tf2
  std_msgs
  rosidl_default_runtime
)

# サービスメッセージの依存を追加
# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
# target_link_libraries(liftup_node_entrypoint "${cpp_typesupport_target}")

# 実行ファイルをインストール
install(TARGETS
  liftup_node_entrypoint
  DESTINATION lib/${PROJECT_NAME}
)

# ヘッダファイルをインストール
install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
