function(axes_add_example name deps)
  file(GLOB_RECURSE sources "${name}/*.cpp" "${name}/*.cu")
  add_executable(${name} ${sources})
  target_link_libraries(${name} PUBLIC ${deps})
endfunction(axes_add_example)

axes_add_example(empty "axes_core")
axes_add_example(cuda_intro "axes_core")
axes_add_example(simd_test "axes_core")
axes_add_example(basic_graphics "axes_gl")
axes_add_example(optim "axes_optim")
axes_add_example(poisson2d "axes_graph;axes_fem")
axes_add_example(poisson3d "axes_fem")
axes_add_example(lattice_poisson2d "axes_gl;axes_fdfv")
axes_add_example(normal "axes_gl")
axes_add_example(graph "axes_graph")

# Homework for DGP
axes_add_example(decimate "axes_gl")
axes_add_example(surface_reconstruct "axes_gl")
axes_add_example(arap_parm "axes_gl")
axes_add_example(cubicstylization "axes_graph")
axes_add_example(geodesic_only_flip "axes_graph")

# FEM
axes_add_example(fem2debug "axes_gl;axes_fem")
axes_add_example(fem3debug "axes_gl;axes_fem")
axes_add_example(fem3newton "axes_gl;axes_fem")
axes_add_example(fem3liu "axes_gl;axes_fem")
axes_add_example(fem_tetra_fd "axes_fem")
axes_add_example(fem_utils "axes_fem;axes_graph")
axes_add_example(fem3_experiment "axes_fem;axes_graph")
axes_add_example(rom_experimental "axes_fem;axes_graph")

# Homework for Nonlinear Numerical PDE:
axes_add_example(char_burgers axes_gl)
axes_add_example(burgers_godunov axes_gl)
axes_add_example(burgers_high_order axes_graph)
