find_package(doctest CONFIG REQUIRED)

function(axes_add_test TARGET_NAME LIBRARIES)
  set(_target ${TARGET_NAME}_test)
  aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME} TEST_SOURCE)
  add_executable(${_target} ${TEST_SOURCE})
  target_link_libraries(${_target} PRIVATE doctest::doctest ${LIBRARIES})
  add_test(NAME test_${TARGET_NAME} COMMAND ${_target})
  ax_set_warn_levels(${_target})
endfunction(axes_add_test)

axes_add_test(math axes)
axes_add_test(fem axes_fem)
axes_add_test(geometry axes_geometry)

