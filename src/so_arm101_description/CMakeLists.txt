cmake_minimum_required(VERSION 3.10...3.27)

project(so_arm101_description)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

# # Install the mesh files from SO101/assets install( DIRECTORY SO101/assets/
# DESTINATION share/${PROJECT_NAME}/meshes FILES_MATCHING PATTERN "*.stl" )

# Install URDF files
install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
  FILES_MATCHING
  PATTERN "*.urdf"
  PATTERN "*.xacro")

# Install other directories
install(
  DIRECTORY meshes config launch rviz
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

# The URDF is generated at build time from the unmodified upstream URDF
# (third_party/SO-ARM100/so101_new_calib.urdf, TheRobotStudio/SO-ARM100 385e8d7, Apache-2.0).
# Only the mesh path prefix is replaced (cmake/gen_urdf.cmake). The generated file is installed under the same
# name and path as before: share/so_arm101_description/urdf/so101_new_calib.urdf
set(GEN_URDF ${CMAKE_CURRENT_BINARY_DIR}/urdf/so101_new_calib.urdf)
add_custom_command(
  OUTPUT ${GEN_URDF}
  COMMAND ${CMAKE_COMMAND} -DIN=${CMAKE_CURRENT_SOURCE_DIR}/third_party/SO-ARM100/so101_new_calib.urdf
          -DOUT=${GEN_URDF} -DSUMS=${CMAKE_CURRENT_SOURCE_DIR}/third_party/SO-ARM100/SHA256SUMS
          -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/gen_urdf.cmake
  DEPENDS third_party/SO-ARM100/so101_new_calib.urdf third_party/SO-ARM100/SHA256SUMS cmake/gen_urdf.cmake
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Generating so101_new_calib.urdf from the upstream URDF"
  VERBATIM)
add_custom_target(gen_urdf ALL DEPENDS ${GEN_URDF})
install(FILES ${GEN_URDF} DESTINATION share/${PROJECT_NAME}/urdf)
# upstream license and source record travel with the package
install(DIRECTORY third_party DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # find_package(ament_clang_format REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # sha256 of the files copied from upstream, and the generated URDF vs the upstream URDF
  add_test(NAME urdf_original_checksums
           COMMAND ${CMAKE_COMMAND} -DPKG_DIR=${CMAKE_CURRENT_SOURCE_DIR}
                   -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/check_checksums.cmake)
  add_test(NAME urdf_generated_matches_original
           COMMAND ${CMAKE_COMMAND} -DPKG_DIR=${CMAKE_CURRENT_SOURCE_DIR} -DGEN=${GEN_URDF}
                   -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/check_generated.cmake)
endif()

ament_package()
