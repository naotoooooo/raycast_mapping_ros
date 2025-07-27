cmake_minimum_required(VERSION 3.8)
project(raycast_mapping_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(raycast_mapping_node
  src/raycast_mapping_node.cpp
  src/raycast_mapping.cpp
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# OpenCVのビルド環境の設定ファイルを読み込む
find_package(OpenCV REQUIRED)

# 依存関係を追加
ament_target_dependencies(raycast_mapping_node
  OpenCV
)

# コード品質を自動的にチェック
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()