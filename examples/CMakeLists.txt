function(axes_add_example name deps)
  file(GLOB_RECURSE sources "${name}/*.cpp")
  add_executable(${name} ${sources})
  target_link_libraries(${name} PUBLIC ${deps})
  ax_set_warn_levels(${name})
endfunction(axes_add_example)

axes_add_example(empty "axes")
axes_add_example(simd_test "axes")
axes_add_example(basic_graphics "axes_gl")
axes_add_example(optim "axes_optim")
axes_add_example(poisson2d "axes_graph;axes_fem")
axes_add_example(poisson3d "axes;axes_fem")
axes_add_example(lattice_poisson2d "axes_gl;axes_fdfv")
axes_add_example(normal "axes_gl")
axes_add_example(graph "axes_graph")

# DGP
axes_add_example(decimate "axes_gl")
axes_add_example(surface_reconstruct "axes_gl")
axes_add_example(arap_parm "axes_gl")
axes_add_example(cubicstylization "axes_graph")

# FEM
axes_add_example(fem2basic "axes_gl;axes_fem")
axes_add_example(fem3basic "axes_gl;axes_fem")
axes_add_example(fem2debug "axes_gl;axes_fem")
axes_add_example(fem3debug "axes_gl;axes_fem")
axes_add_example(fem3newton "axes_gl;axes_fem")
axes_add_example(fem3liu "axes_gl;axes_fem")
axes_add_example(fem_tetra_fd "axes_fem")
axes_add_example(fem_utils "axes_fem;axes_graph")

# Homework for Nonlinear Numerical PDE:
axes_add_example(char_burgers axes_gl)
axes_add_example(burgers_godunov axes_gl)
axes_add_example(burgers_high_order axes_graph)