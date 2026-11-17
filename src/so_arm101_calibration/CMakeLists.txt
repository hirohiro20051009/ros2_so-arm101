cmake_minimum_required(VERSION 3.10...3.27)
project(so_arm101_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
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

# Pure Python library of the zero-offset calibration (no ROS node is started from it).
# The Python package directory so_arm101_calibration/ is separate from include/so_arm101_calibration/.
ament_python_install_package(${PROJECT_NAME})

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

  # Python: only flake8 and pep257 (not ament_lint_auto: the C++ files do not have the standard copyright header).
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  ament_flake8()
  ament_pep257()
  # Do not autoload third-party pytest plugins (see so_arm101_ik/CMakeLists.txt).
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} 1)
  ament_add_pytest_test(test_zero_offset test/test_zero_offset.py
                        ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(test_zero_offset_static test/test_zero_offset_static.py
                        ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(test_zero_offset_record test/test_zero_offset_record.py
                        ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(test_zero_offset_record_static test/test_zero_offset_record_static.py
                        ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
endif()

ament_package()
