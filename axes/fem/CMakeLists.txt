# Library axes-core
file(GLOB_RECURSE AX_FEM_SOURCE CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/source/**.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/**.hpp)

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
    axes
    axes_geometry)

ax_set_warn_levels(axes_fem)
target_compile_options(axes_fem PRIVATE ${AX_CXX_FLAGS})
