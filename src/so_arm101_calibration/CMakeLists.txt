cmake_minimum_required(VERSION 3.10...3.27)
project(so_arm101_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(so_arm101_hardware REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(so_arm101_calib_core SHARED
  src/calib_write_bus.cpp
  src/eeprom_backup.cpp
  src/eeprom_ops.cpp
  src/restore.cpp
  src/measure.cpp
  src/apply.cpp)
target_include_directories(so_arm101_calib_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(so_arm101_calib_core so_arm101_hardware)
target_link_libraries(so_arm101_calib_core yaml-cpp)

add_executable(so_arm101_calib src/main.cpp)
target_link_libraries(so_arm101_calib so_arm101_calib_core)
ament_target_dependencies(so_arm101_calib ament_index_cpp)

install(TARGETS so_arm101_calib_core LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS so_arm101_calib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # 偽サーボ（pty）は so_arm101_hardware の test/fake_servo.hpp を使う（実機・実ポートは開かない）
  ament_add_gtest(test_calibration test/test_calibration.cpp)
  ament_add_gtest(test_measure_apply test/test_measure_apply.cpp)
  target_link_libraries(test_measure_apply so_arm101_calib_core util)
  target_include_directories(test_measure_apply PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../so_arm101_hardware/test)
  target_compile_definitions(test_measure_apply PRIVATE
    CALIBRATION_YAML="${CMAKE_CURRENT_SOURCE_DIR}/../so_arm101_hardware/test/data/follower_calibration_fixture_20260907.yaml")
  target_link_libraries(test_calibration so_arm101_calib_core util)
  target_include_directories(test_calibration PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../so_arm101_hardware/test)
  target_compile_definitions(test_calibration PRIVATE
    PACKAGE_XML="${CMAKE_CURRENT_SOURCE_DIR}/package.xml"
    PRODUCTION_YAML="${CMAKE_CURRENT_SOURCE_DIR}/../so_arm101_hardware/config/follower_calibration.yaml"
    EEPROM_AFTER_C3_JSON="${CMAKE_CURRENT_SOURCE_DIR}/../../calibration_records/2026-11-11/eeprom_after_C3.json"
    CALIBRATION_YAML="${CMAKE_CURRENT_SOURCE_DIR}/../so_arm101_hardware/test/data/follower_calibration_fixture_20260907.yaml")
endif()

ament_package()
