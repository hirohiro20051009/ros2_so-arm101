# Generate the installable URDF from the unmodified upstream URDF (cmake -P script).
#   -DIN=<third_party/SO-ARM100/so101_new_calib.urdf>  -DOUT=<generated file>  -DSUMS=<SHA256SUMS>
# The only change is the mesh path prefix; one XML comment line is added after the XML declaration.
# Any unexpected input stops the build.
foreach(v IN ITEMS IN OUT SUMS)
  if(NOT DEFINED ${v})
    message(FATAL_ERROR "gen_urdf.cmake: -D${v}= is required")
  endif()
endforeach()

set(ORIG_COMMIT "385e8d7c68e24945df6c60d9bd68837a4b7411ae")
set(FROM "filename=\"assets/")
set(TO "filename=\"package://so_arm101_description/meshes/")
set(EXPECTED_REFS 34)

# 1. the input must be the recorded file (sha256 in SHA256SUMS)
file(SHA256 "${IN}" actual)
file(STRINGS "${SUMS}" sums_lines)
set(expected "")
foreach(line IN LISTS sums_lines)
  if(line MATCHES "^([0-9a-f]+)  third_party/SO-ARM100/so101_new_calib\\.urdf$")
    set(expected "${CMAKE_MATCH_1}")
  endif()
endforeach()
if(expected STREQUAL "")
  message(FATAL_ERROR "gen_urdf.cmake: no entry for the URDF in ${SUMS}")
endif()
if(NOT actual STREQUAL expected)
  message(FATAL_ERROR "gen_urdf.cmake: sha256 of ${IN} is ${actual}, recorded ${expected}")
endif()

# 2. the number of references to replace
file(READ "${IN}" content)
string(REGEX MATCHALL "filename=\"assets/" refs "${content}")
list(LENGTH refs n_refs)
string(REGEX MATCHALL "assets" any_assets "${content}")
list(LENGTH any_assets n_any)
if(NOT n_refs EQUAL EXPECTED_REFS OR NOT n_any EQUAL EXPECTED_REFS)
  message(FATAL_ERROR "gen_urdf.cmake: expected ${EXPECTED_REFS} 'assets' references, found ${n_refs} (all 'assets': ${n_any})")
endif()

# 3. replace, add the source comment after the XML declaration
string(REPLACE "${FROM}" "${TO}" content "${content}")
set(decl "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n")
string(FIND "${content}" "${decl}" decl_pos)
if(NOT decl_pos EQUAL 0)
  message(FATAL_ERROR "gen_urdf.cmake: unexpected XML declaration in ${IN}")
endif()
set(comment "<!-- GENERATED at build time (cmake/gen_urdf.cmake) from third_party/SO-ARM100/so101_new_calib.urdf (TheRobotStudio/SO-ARM100 ${ORIG_COMMIT}, Apache-2.0, sha256 ${actual}). Only change: mesh filename prefix assets/ -> package://so_arm101_description/meshes/ -->\n")
string(REPLACE "${decl}" "${decl}${comment}" content "${content}")

get_filename_component(out_dir "${OUT}" DIRECTORY)
file(MAKE_DIRECTORY "${out_dir}")
file(WRITE "${OUT}" "${content}")
message(STATUS "gen_urdf.cmake: wrote ${OUT} (${n_refs} mesh paths replaced)")
