cmake_minimum_required(VERSION 3.10...3.27)
project(so_arm101_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in further dependencies
# manually. find_package(<dependency> REQUIRED)

# Install other directories
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
        # FILES_MATCHING PATTERN "*.launch.py"
)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  # the following line skips the linter which checks for copyrights comment the
  # line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE) the following line skips cpplint (only
  # works in a git repo) comment the line when this package is in a git repo and
  # when a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # check of undeclared launch arguments (no node is started).
  # A pytest plugin installed in the user site (e.g. anyio in ~/.local) can be incompatible with the
  # system pytest and make "python3 -m pytest" fail; ament_add_pytest_test() then silently skips the
  # test. Do not autoload third-party pytest plugins (at configure time and when the test runs).
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} 1)
  ament_add_pytest_test(test_launch_arg_check test/test_launch_arg_check.py
                        ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
endif()

ament_package()
