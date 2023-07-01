function(axes_add_example name deps)
  file(GLOB_RECURSE sources "${name}/*.cpp")
  add_executable(${name} ${sources})
  target_link_libraries(${name} PRIVATE axes ${deps})
  axes_set_warn_levels(${name})
endfunction(axes_add_example)

find_package(benchmark CONFIG REQUIRED)

axes_add_example(ecs "")
axes_add_example(guitutor "")
axes_add_example(null "")
axes_add_example(bench-math "benchmark::benchmark;benchmark::benchmark_main")
