macro(axes_add_example name)
  file(GLOB_RECURSE sources "${name}/*.cpp" "${name}/*.cu")
  add_executable(${name} ${sources})
  target_link_libraries(${name} PUBLIC ${ARGN})
  ax_target_apply_flag(${name} PRIVATE)
endmacro(axes_add_example)

axes_add_example(empty "axes_core")
axes_add_example(basic_graphics "axes_gl")
axes_add_example(arap_parm "axes_gl")
axes_add_example(optim "axes_optim")
axes_add_example(poisson2d "axes_graph;axes_fem")
axes_add_example(poisson3d "axes_fem")
axes_add_example(cubic_solver "axes_core")
axes_add_example(graph "axes_graph")
axes_add_example(strides_cpu "axes_core")
axes_add_example(parallel "axes_core")
axes_add_example(block_precond_tests "axes_core")
axes_add_example(csr_compress_tests "axes_core;axes_fem")
axes_add_example(fem_gather_tests "axes_core;axes_fem")
axes_add_example(unconst_quad_opt "axes_core;axes_optim2")

# FEM
axes_add_example(fem2debug "axes_gl;axes_fem")
axes_add_example(fem3debug "axes_gl;axes_fem")
axes_add_example(fem_benchmark "axes_gl;axes_fem")
axes_add_example(fem_tetra_fd "axes_fem")
axes_add_example(fem_tetra_fd2 "axes_fem")
axes_add_example(fem_utils "axes_fem;axes_graph")
axes_add_example(fem_poisson "axes_fem")
axes_add_example(fem_backward_euler "axes_fem;axes_gl")

axes_add_example(poisson_bench "axes_fem;axes_gl")

# CUDA Support
if(AX_ENABLE_CUDA)
  axes_add_example(cuda_intro "axes_core")
  axes_add_example(fem_tetra_fd_gpu "axes_fem")
  axes_add_example(cu_admm "axes_gl;axes_fem")
  axes_add_example(test_cu_block_mm "axes_core")
endif()

find_package(benchmark CONFIG QUIET)

if(benchmark_FOUND)
  axes_add_example(optimizer_benchmark "benchmark::benchmark;axes_optim")
  axes_add_example(bufv_benchmark "benchmark::benchmark;axes_core")
endif()
