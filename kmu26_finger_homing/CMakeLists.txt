cmake_minimum_required(VERSION 3.10)
project(kmu26_finger_homing)

find_package(ament_cmake REQUIRED)
find_package(audio_common_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(finger_audio_estimator src/finger_audio_estimator.cpp)
ament_target_dependencies(finger_audio_estimator
  audio_common_msgs rclcpp std_msgs)

add_executable(finger_frequency_selector src/finger_frequency_selector.cpp)
ament_target_dependencies(finger_frequency_selector
  audio_common_msgs rclcpp std_msgs)

add_executable(finger_homing_controller src/finger_homing_controller.cpp)
ament_target_dependencies(finger_homing_controller
  geometry_msgs mavros_msgs nav_msgs rclcpp std_msgs)
target_include_directories(finger_homing_controller PRIVATE ${EIGEN3_INCLUDE_DIRS})

install(TARGETS
  finger_audio_estimator
  finger_frequency_selector
  finger_homing_controller
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})
install(FILES README.md
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_package_contract test/test_package_contract.py)
endif()

ament_package()
