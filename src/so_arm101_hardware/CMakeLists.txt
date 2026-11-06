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

# Create the shared library for the hardware interface plugin
add_library(so101_hardware_bridge SHARED src/so101_hardware_bridge.cpp)

# Link the library against its dependencies
ament_target_dependencies(so101_hardware_bridge rclcpp hardware_interface
                          pluginlib)

# Feetech STS3215 を直接読む SystemInterface（読み取り専用）。LeRobot には依存しない。
add_library(so_arm101_sts3215 SHARED
  src/serial_port.cpp
  src/sts3215_bus.cpp
  src/calibration.cpp
  src/sts3215_system.cpp)
ament_target_dependencies(so_arm101_sts3215 rclcpp hardware_interface pluginlib)
target_link_libraries(so_arm101_sts3215 yaml-cpp)

# Install the compiled library to the lib directory
install(
  TARGETS so101_hardware_bridge so_arm101_sts3215
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install the header files and the calibration yaml
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Register the plugin with pluginlib
pluginlib_export_plugin_description_file(hardware_interface
                                         so101_hardware_bridge_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # 変換の正解表（LeRobot 0.4.2 の _normalize 由来）との一致、パケット/エラー処理（pty 上の偽サーボ）
  ament_add_gtest(test_conversion test/test_conversion.cpp)
  target_link_libraries(test_conversion so_arm101_sts3215)
  target_compile_definitions(test_conversion PRIVATE
    CALIBRATION_YAML="${CMAKE_CURRENT_SOURCE_DIR}/config/follower_calibration.yaml"
    GOLDEN_CSV="${CMAKE_CURRENT_SOURCE_DIR}/test/data/conversion_golden.csv")

  ament_add_gtest(test_sts3215_bus test/test_sts3215_bus.cpp)
  target_link_libraries(test_sts3215_bus so_arm101_sts3215 util)
endif()

ament_package()
