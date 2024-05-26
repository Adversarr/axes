# Library axes-core
file(GLOB_RECURSE _SOURCE CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/source/**.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/**.hpp)

if (${AX_BUILD_SHARED})
  add_library(axes_xpbd SHARED ${_SOURCE})
else()
  add_library(axes_xpbd STATIC ${_SOURCE})
endif()

target_include_directories(axes_xpbd
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(axes_xpbd
  PUBLIC
    axes_geometry
    axes_optim
)

ax_target_apply_flag(axes_xpbd PRIVATE)
