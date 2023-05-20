function(axes_add_example name deps)
    file(GLOB_RECURSE sources "${name}/*.cpp")
    add_executable(${name} ${sources})
    target_link_libraries(${name} PRIVATE ${deps})
    axes_set_warn_levels(${name})
endfunction(axes_add_example)


axes_add_example(ecs "AxesCore")
axes_add_example(guitutor "AxesCore;AxesGui")
