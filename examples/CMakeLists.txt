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
axes_add_example(lattice_poisson2d "axes_gl;axes_fdfv")
axes_add_example(cubic_solver "axes_core")
axes_add_example(graph "axes_graph")
axes_add_example(strides_cpu "axes_core")
axes_add_example(parallel "axes_core")

# admm
axes_add_example(cadmm "axes_gl;axes_xpbd")
axes_add_example(admm_sph "axes_gl;axes_xpbd")
axes_add_example(springs "axes_gl;axes_optim")

# FEM
axes_add_example(fem2debug "axes_gl;axes_fem")
axes_add_example(fem3debug "axes_gl;axes_fem")
axes_add_example(fem_benchmark "axes_gl;axes_fem")
axes_add_example(fem_tetra_fd "axes_fem")
axes_add_example(fem_utils "axes_fem;axes_graph")

axes_add_example(poisson_bench "axes_fem;axes_gl")

# CUDA Support
if(AX_ENABLE_CUDA)
  axes_add_example(cuda_intro "axes_core")
  axes_add_example(fem_tetra_fd_gpu "axes_fem")
  axes_add_example(map_to_cuda "axes_core")
  axes_add_example(cu_admm "axes_gl;axes_fem")
  axes_add_example(memory "axes_core")
  axes_add_example(test_cu_block_mm "axes_core")
endif()

find_package(benchmark CONFIG QUIET)


if(benchmark_FOUND)
  axes_add_example(optimizer_benchmark "benchmark::benchmark;axes_optim")
  axes_add_example(bufv_benchmark "benchmark::benchmark;axes_core")
endif()
