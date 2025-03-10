#!/bin/bash

# ビルドディレクトリとインストールディレクトリを削除
rm -rf log/ build/ install/

# colcon build を実行し、エラーが発生した場合はスクリプトを終了
colcon build
if [ $? -ne 0 ]; then
  echo "Build failed"
  exit 1
fi

# インストールディレクトリのセットアップスクリプトをソースし、エラーが発生した場合はスクリプトを終了
source install/setup.bash
if [ $? -ne 0 ]; then
  echo "Failed to source setup.bash"
  exit 1
fi

# ランチファイルを実行
ros2 launch liftup liftup_launch.py
if [ $? -ne 0 ]; then
  echo "Failed to launch liftup_launch.py"
  exit 1
fi