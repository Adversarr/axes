file(GLOB_RECURSE AX_FEM_SOURCE CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/source/**.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/**.hpp)

if (AX_ENABLE_CUDA)
  file(GLOB_RECURSE AX_FEM_CUDA_SOURCE CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/source/**.cu
    ${CMAKE_CURRENT_SOURCE_DIR}/include/**.cuh)
  list(APPEND AX_FEM_SOURCE ${AX_FEM_CUDA_SOURCE})
  message(STATUS "CUDA enabled for axes-fem: ${AX_FEM_CUDA_SOURCE}")
endif()

if (${AX_BUILD_SHARED})
  add_library(axes_fem SHARED ${AX_FEM_SOURCE})
else()
  add_library(axes_fem STATIC ${AX_FEM_SOURCE})
endif()

target_include_directories(axes_fem
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(axes_fem
  PUBLIC
    axes_core
    axes_optim
    axes_geometry)

ax_target_apply_flag(axes_fem PRIVATE)
