find_package(libigl CONFIG REQUIRED)

# Library axes-core
file(GLOB_RECURSE AX_GEOMETRY_SOURCE CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/source/**.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/**.hpp)

if (${AX_BUILD_SHARED})
  add_library(axes_geometry SHARED ${AX_GEOMETRY_SOURCE})
else()
  add_library(axes_geometry STATIC ${AX_GEOMETRY_SOURCE})
endif()

target_include_directories(axes_geometry
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(axes_geometry
  PUBLIC
    axes
    igl::igl_core
)

ax_set_warn_levels(axes_geometry)
target_compile_options(axes_geometry PRIVATE ${AX_CXX_FLAGS})
