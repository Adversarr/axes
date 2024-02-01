function(axes_add_example name deps)
  file(GLOB_RECURSE sources "${name}/*.cpp")
  add_executable(${name} ${sources})
  target_link_libraries(${name} PRIVATE axes ${deps})
  axes_set_warn_levels(${name})
endfunction(axes_add_example)

axes_add_example(empty "")
axes_add_example(basic_graphics axes_gui)
