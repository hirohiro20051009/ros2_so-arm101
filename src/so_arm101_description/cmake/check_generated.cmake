# Test: the generated URDF equals the upstream URDF except for the mesh prefix and the one added comment line,
# and every referenced mesh exists.
#   -DPKG_DIR=<package source dir>  -DGEN=<generated URDF>
if(NOT EXISTS "${GEN}")
  message(FATAL_ERROR "generated URDF not found (build first): ${GEN}")
endif()
file(READ "${PKG_DIR}/third_party/SO-ARM100/so101_new_calib.urdf" orig)
file(READ "${GEN}" gen)

# remove the added comment line (2nd line)
string(FIND "${gen}" "\n" p1)
math(EXPR s1 "${p1}+1")
string(SUBSTRING "${gen}" ${s1} -1 rest)
string(FIND "${rest}" "\n" p2)
math(EXPR s2 "${p2}+1")
string(SUBSTRING "${rest}" 0 ${p2} added)
string(SUBSTRING "${rest}" ${s2} -1 rest2)
if(NOT added MATCHES "^<!-- GENERATED at build time")
  message(FATAL_ERROR "2nd line of the generated URDF is not the source comment: ${added}")
endif()
string(SUBSTRING "${gen}" 0 ${s1} first_line)
set(gen_wo_comment "${first_line}${rest2}")

# reverse replacement
string(REPLACE "filename=\"package://so_arm101_description/meshes/" "filename=\"assets/" back "${gen_wo_comment}")
if(NOT back STREQUAL orig)
  message(FATAL_ERROR "generated URDF differs from upstream beyond the mesh prefix and the comment line")
endif()

# 34 references, all existing
string(REGEX MATCHALL "package://so_arm101_description/meshes/[A-Za-z0-9_]+\\.stl" refs "${gen}")
list(LENGTH refs n)
if(NOT n EQUAL 34)
  message(FATAL_ERROR "expected 34 mesh references, found ${n}")
endif()
list(REMOVE_DUPLICATES refs)
foreach(r IN LISTS refs)
  string(REPLACE "package://so_arm101_description/" "" rel "${r}")
  if(NOT EXISTS "${PKG_DIR}/${rel}")
    message(FATAL_ERROR "referenced mesh does not exist: ${rel}")
  endif()
endforeach()
list(LENGTH refs n_unique)
message(STATUS "urdf_generated_matches_original: OK (34 references, ${n_unique} distinct meshes)")
