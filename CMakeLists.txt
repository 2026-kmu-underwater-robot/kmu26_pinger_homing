cmake_minimum_required(VERSION 3.8)
project(kmu26_mission_fsm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(ground_truth_buoy_fsm src/ground_truth_buoy_fsm.cpp)
target_compile_features(ground_truth_buoy_fsm PUBLIC cxx_std_17)
ament_target_dependencies(ground_truth_buoy_fsm rclcpp geometry_msgs mavros_msgs std_msgs)

add_executable(pinger_homing_controller src/pinger_homing_controller.cpp)
target_compile_features(pinger_homing_controller PUBLIC cxx_std_17)
ament_target_dependencies(pinger_homing_controller rclcpp geometry_msgs mavros_msgs std_msgs)

add_executable(mission_rviz_visualizer src/mission_rviz_visualizer.cpp)
target_compile_features(mission_rviz_visualizer PUBLIC cxx_std_17)
ament_target_dependencies(mission_rviz_visualizer rclcpp geometry_msgs std_msgs visualization_msgs)

add_executable(check_ground_truth_buoy_fsm_cpp test/check_ground_truth_buoy_fsm_cpp.cpp)
target_compile_features(check_ground_truth_buoy_fsm_cpp PUBLIC cxx_std_17)
target_include_directories(check_ground_truth_buoy_fsm_cpp PRIVATE src)

if(BUILD_TESTING)
  enable_testing()
  add_test(
    NAME ground_truth_buoy_fsm_cpp
    COMMAND check_ground_truth_buoy_fsm_cpp ${CMAKE_CURRENT_SOURCE_DIR}/config/tank_current_scene.xml
  )
endif()

install(TARGETS
  ground_truth_buoy_fsm
  pinger_homing_controller
  mission_rviz_visualizer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
