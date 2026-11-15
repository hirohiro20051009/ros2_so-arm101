cmake_minimum_required(VERSION 3.10...3.27)
project(so_arm101_ik)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Pure Python library (numpy only, no ROS dependency at run time).
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # A pytest plugin installed in the user site (e.g. anyio in ~/.local) can be incompatible with the
  # system pytest and make "python3 -m pytest" fail; ament_add_pytest_test() then silently skips the
  # test. Do not autoload third-party pytest plugins (at configure time and when the test runs).
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} 1)
  foreach(test_name
          test_constants test_fk_chain test_ik_roundtrip test_ik_roll test_ik_limits
          test_model_check test_moveit_fixture test_ik_model_check_static
          test_diff_mode test_diff_mode_safety)
    ament_add_pytest_test(${test_name} test/${test_name}.py
                          ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  endforeach()
endif()

ament_package()
