cmake_minimum_required(VERSION 3.8)
project(kmu26_vision_mission_fsm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(hit25_auv_ros2_msg REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(observation_mission_fsm src/observation_mission_fsm.cpp)
target_compile_features(observation_mission_fsm PUBLIC cxx_std_17)
target_include_directories(observation_mission_fsm PRIVATE src)
ament_target_dependencies(
  observation_mission_fsm rclcpp geometry_msgs mavros_msgs nav_msgs std_msgs
  std_srvs hit25_auv_ros2_msg)
target_link_libraries(observation_mission_fsm Eigen3::Eigen)

add_executable(bbox_controller_node src/bbox_controller_node.cpp)
target_compile_features(bbox_controller_node PUBLIC cxx_std_17)
ament_target_dependencies(bbox_controller_node rclcpp std_msgs mavros_msgs)

add_executable(mission_state_machine_node src/mission_state_machine_node.cpp)
target_compile_features(mission_state_machine_node PUBLIC cxx_std_17)
ament_target_dependencies(
  mission_state_machine_node geometry_msgs rclcpp std_msgs mavros_msgs nav_msgs)

add_executable(ground_truth_buoy_fsm src/ground_truth_buoy_fsm.cpp)
target_compile_features(ground_truth_buoy_fsm PUBLIC cxx_std_17)
ament_target_dependencies(ground_truth_buoy_fsm rclcpp geometry_msgs mavros_msgs nav_msgs std_msgs)

add_executable(mission_rviz_visualizer src/mission_rviz_visualizer.cpp)
target_compile_features(mission_rviz_visualizer PUBLIC cxx_std_17)
ament_target_dependencies(
  mission_rviz_visualizer rclcpp geometry_msgs nav_msgs std_msgs visualization_msgs
  hit25_auv_ros2_msg)

add_executable(check_ground_truth_buoy_fsm_cpp test/check_ground_truth_buoy_fsm_cpp.cpp)
target_compile_features(check_ground_truth_buoy_fsm_cpp PUBLIC cxx_std_17)
target_include_directories(check_ground_truth_buoy_fsm_cpp PRIVATE src)

if(BUILD_TESTING)
  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME ground_truth_buoy_fsm_cpp
    COMMAND check_ground_truth_buoy_fsm_cpp ${CMAKE_CURRENT_SOURCE_DIR}/config/tank_current_scene.xml)
  add_test(NAME observation_mission_runtime
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_observation_mission_runtime.py
      --controller $<TARGET_FILE:observation_mission_fsm>)
  add_test(NAME observation_boundary_rejection
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_observation_mission_runtime.py
      --controller $<TARGET_FILE:observation_mission_fsm> --boundary-rejection)
  add_test(NAME observation_capture_retry
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_observation_mission_runtime.py
      --controller $<TARGET_FILE:observation_mission_fsm> --capture-retry)
  add_test(NAME observation_stick_rake_flow
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_observation_mission_runtime.py
      --controller $<TARGET_FILE:observation_mission_fsm> --stick-rake-flow)
  add_test(NAME delegated_vision_handoff
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_delegated_vision_handoff.py
      --controller $<TARGET_FILE:observation_mission_fsm>)
  add_test(NAME buoy_vision_core
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_buoy_vision_core.py)
  add_test(NAME vision_mission_controller_runtime
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_vision_mission_controller_runtime.py
      --controller $<TARGET_FILE:mission_state_machine_node>)
  add_test(NAME vision_mission_controller_dry_run
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_vision_mission_controller_runtime.py
      --controller $<TARGET_FILE:mission_state_machine_node> --dry-run)
  set_tests_properties(
    observation_mission_runtime observation_boundary_rejection observation_capture_retry
    observation_stick_rake_flow delegated_vision_handoff
    vision_mission_controller_runtime vision_mission_controller_dry_run
    PROPERTIES TIMEOUT 30)
endif()

install(TARGETS
  observation_mission_fsm bbox_controller_node mission_state_machine_node
  ground_truth_buoy_fsm mission_rviz_visualizer
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/buoy_vision_core.py
  scripts/fsm_web_gui.py
  scripts/real_vehicle_preflight.sh
  scripts/check_yolo_env.py
  scripts/setup_laptop_yolo_env.sh
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/yolo_buoy_detector.py
  DESTINATION lib/${PROJECT_NAME} RENAME yolo_buoy_detector)
install(PROGRAMS scripts/run_yolo_buoy_detector.sh
  DESTINATION lib/${PROJECT_NAME} RENAME run_yolo_buoy_detector)

install(DIRECTORY launch config rviz web DESTINATION share/${PROJECT_NAME})
install(DIRECTORY src/vision_buoy DESTINATION share/${PROJECT_NAME}/algorithms
  FILES_MATCHING PATTERN "*.md" PATTERN "*.hpp")
install(FILES requirements-yolo.txt DESTINATION share/${PROJECT_NAME})

ament_package()
