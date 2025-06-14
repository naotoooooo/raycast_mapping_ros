cmake_minimum_required(VERSION 3.8)
project(raycast_mapping_ros)

add_compile_options(-std=c++17 -O2 -g)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  pcl_ros
  roscpp
  sensor_msgs
  tf2_eigen
  tf2_ros
)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(raycast_mapping_node src/raycast_mapping.cpp)
target_link_libraries(raycast_mapping_node ${catkin_LIBRARIES})

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()

install(TARGETS raycast_mapping_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)


# OpenCVのビルド環境の設定ファイルを読み込む
find_package(OpenCV REQUIRED)

# OpenCV関係のインクルードディレクトリのパスを設定
include_directories(
    ${OpenCV_INCLUDE_DIRS}
)

# リンクするライブラリを読み込む
target_link_libraries(raycast_mapping_node
    ${OpenCV_LIBRARIES}
)