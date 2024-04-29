# Library axes-core
file(GLOB_RECURSE AX_OPTIM_SOURCE CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/source/**.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/**.hpp)

if (${AX_BUILD_SHARED})
  add_library(axes_optim SHARED ${AX_OPTIM_SOURCE})
else()
  add_library(axes_optim STATIC ${AX_OPTIM_SOURCE})
endif()

target_include_directories(axes_optim
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(axes_optim
  PUBLIC
    axes
)

ax_target_apply_flag(axes_optim PRIVATE)
