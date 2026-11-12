cmake_minimum_required(VERSION 3.10...3.27)
project(so_arm101_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(yaml-cpp REQUIRED)

# Add the include directory for the header file
include_directories(include)

# Feetech STS3215 を直接読む SystemInterface（読み取り専用）。LeRobot には依存しない（基準の値・式は LeRobot 0.4.2 由来。docs/third_party.md）。
add_library(so_arm101_sts3215 SHARED
  src/serial_port.cpp
  src/sts3215_bus.cpp
  src/sts3215_write_bus.cpp
  src/calibration.cpp
  src/servo_config.cpp
  src/sts3215_system.cpp)
ament_target_dependencies(so_arm101_sts3215 rclcpp hardware_interface pluginlib)
target_link_libraries(so_arm101_sts3215 yaml-cpp)

# Install the compiled library to the lib directory
install(
  TARGETS so_arm101_sts3215
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install the header files and the calibration yaml
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Register the plugin with pluginlib
pluginlib_export_plugin_description_file(hardware_interface
                                         so_arm101_hardware_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # 変換の正解表（LeRobot 0.4.2 の _normalize 由来）との一致、パケット/エラー処理（pty 上の偽サーボ）
  ament_add_gtest(test_conversion test/test_conversion.cpp)
  target_link_libraries(test_conversion so_arm101_sts3215)
  target_compile_definitions(test_conversion PRIVATE
    CALIBRATION_YAML="${CMAKE_CURRENT_SOURCE_DIR}/test/data/follower_calibration_fixture_20260907.yaml"
    PRODUCTION_YAML="${CMAKE_CURRENT_SOURCE_DIR}/config/follower_calibration.yaml"
    GOLDEN_CSV="${CMAKE_CURRENT_SOURCE_DIR}/test/data/conversion_golden.csv"
    UNNORMALIZE_GOLDEN_CSV="${CMAKE_CURRENT_SOURCE_DIR}/test/data/unnormalize_golden.csv")

  ament_add_gtest(test_servo_config test/test_servo_config.cpp)
  target_link_libraries(test_servo_config so_arm101_sts3215)

  ament_add_gtest(test_sts3215_bus test/test_sts3215_bus.cpp)
  target_link_libraries(test_sts3215_bus so_arm101_sts3215 util)

  # 段階2b-3b: 書き込みバス、書き込みモードのドライバ（偽サーボ pty）、ソース走査
  ament_add_gtest(test_bus_stale test/test_bus_stale.cpp)
  target_link_libraries(test_bus_stale so_arm101_sts3215 util)

  ament_add_gtest(test_write_bus test/test_write_bus.cpp)
  target_link_libraries(test_write_bus so_arm101_sts3215 util)
  target_compile_definitions(test_write_bus PRIVATE PACKAGE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

  ament_add_gtest(test_sts3215_system test/test_sts3215_system.cpp)
  target_link_libraries(test_sts3215_system so_arm101_sts3215 util)
  target_compile_definitions(test_sts3215_system PRIVATE
    CALIBRATION_YAML="${CMAKE_CURRENT_SOURCE_DIR}/test/data/follower_calibration_fixture_20260907.yaml"
    JOINT_LIMITS_YAML="${CMAKE_CURRENT_SOURCE_DIR}/../so_arm101_moveit_config/config/joint_limits.yaml"
    CONTROLLERS_YAML="${CMAKE_CURRENT_SOURCE_DIR}/../so_arm101_controller/config/controllers.yaml")
endif()

# 他のパッケージ（so_arm101_calibration など）が so_arm101_sts3215（Sts3215Bus / Sts3215WriteBus /
# calibration など）を使えるようにする。公開ヘッダ（sts3215_system.hpp など）が
# rclcpp / rclcpp_lifecycle / hardware_interface / std_msgs / sensor_msgs のヘッダを include するので、
# 依存も export する（yaml-cpp はライブラリがリンクしている）。
ament_export_include_directories(include)
ament_export_libraries(so_arm101_sts3215)
ament_export_dependencies(rclcpp rclcpp_lifecycle hardware_interface std_msgs sensor_msgs yaml-cpp)

ament_package()
