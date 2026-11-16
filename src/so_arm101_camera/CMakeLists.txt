cmake_minimum_required(VERSION 3.10...3.27)
project(so_arm101_camera)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Pure Python library (no ROS node is started from it).
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Do not autoload third-party pytest plugins (see so_arm101_ik/CMakeLists.txt).
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} 1)
  ament_add_pytest_test(test_camera_config test/test_camera_config.py
                        ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(test_tag_mount test/test_tag_mount.py
                        ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(test_record test/test_record.py
                        ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(test_record_static test/test_record_static.py
                        ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(test_camera_record_static test/test_camera_record_static.py
                        ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
endif()

ament_package()
