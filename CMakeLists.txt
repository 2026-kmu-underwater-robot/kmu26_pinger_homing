cmake_minimum_required(VERSION 3.8)
project(kmu26_mission_fsm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(hit25_auv_ros2_msg REQUIRED)

add_executable(ground_truth_buoy_fsm src/ground_truth_buoy_fsm.cpp)
target_compile_features(ground_truth_buoy_fsm PUBLIC cxx_std_17)
ament_target_dependencies(ground_truth_buoy_fsm rclcpp geometry_msgs mavros_msgs nav_msgs std_msgs)

add_executable(pinger_homing_controller src/pinger_homing_controller.cpp)
target_compile_features(pinger_homing_controller PUBLIC cxx_std_17)
ament_target_dependencies(pinger_homing_controller rclcpp geometry_msgs mavros_msgs std_msgs)

add_executable(rc_override_mux src/rc_override_mux.cpp)
target_compile_features(rc_override_mux PUBLIC cxx_std_17)
ament_target_dependencies(rc_override_mux rclcpp mavros_msgs std_msgs)

add_executable(mission_rviz_visualizer src/mission_rviz_visualizer.cpp)
target_compile_features(mission_rviz_visualizer PUBLIC cxx_std_17)
ament_target_dependencies(mission_rviz_visualizer rclcpp geometry_msgs nav_msgs std_msgs visualization_msgs)

add_executable(check_ground_truth_buoy_fsm_cpp test/check_ground_truth_buoy_fsm_cpp.cpp)
target_compile_features(check_ground_truth_buoy_fsm_cpp PUBLIC cxx_std_17)
target_include_directories(check_ground_truth_buoy_fsm_cpp PRIVATE src)

if(BUILD_TESTING)
  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(
    NAME ground_truth_buoy_fsm_cpp
    COMMAND check_ground_truth_buoy_fsm_cpp ${CMAKE_CURRENT_SOURCE_DIR}/config/tank_current_scene.xml
  )
  add_test(
    NAME single_hydrophone_homing_math
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_single_hydrophone_homing_math.py
  )
  add_test(
    NAME pinger_homing_dry_run
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pinger_homing_dry_run.py
      --controller ${CMAKE_CURRENT_SOURCE_DIR}/src/pinger_homing/single_hydrophone_homing_controller.py
  )
  add_test(
    NAME rc_override_mux_runtime
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_rc_override_mux_runtime.py
      --mux $<TARGET_FILE:rc_override_mux>
  )
  set_tests_properties(single_hydrophone_homing_math PROPERTIES TIMEOUT 30)
  set_tests_properties(pinger_homing_dry_run PROPERTIES TIMEOUT 15)
  set_tests_properties(rc_override_mux_runtime PROPERTIES TIMEOUT 15)
endif()

install(TARGETS
  ground_truth_buoy_fsm
  pinger_homing_controller
  rc_override_mux
  mission_rviz_visualizer
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/pinger_homing/single_hydrophone_homing_controller.py
  scripts/fsm_web_gui.py
  scripts/real_vehicle_preflight.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  src/pinger_homing/single_hydrophone_homing_math.py
  PINGER_HOMING_ARCHITECTURE_KO.md
  hydrophone_fork.repos
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  src/pinger_homing
  DESTINATION share/${PROJECT_NAME}/algorithms
  FILES_MATCHING
    PATTERN "*.md"
    PATTERN "__pycache__" EXCLUDE
)

install(DIRECTORY
  launch
  config
  rviz
  web
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
