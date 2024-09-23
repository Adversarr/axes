# Library axes-core
file(GLOB_RECURSE AX_OPTIM2_SOURCE CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/source/**.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/**.hpp)

if (${AX_BUILD_SHARED})
  add_library(axes_optim2 SHARED ${AX_OPTIM2_SOURCE})
else()
  add_library(axes_optim2 STATIC ${AX_OPTIM2_SOURCE})
endif()

target_include_directories(axes_optim2
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(axes_optim2
  PUBLIC
    axes_core
)

ax_target_apply_flag(axes_optim2 PRIVATE)
