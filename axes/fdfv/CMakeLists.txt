# Library axes-core
file(GLOB_RECURSE AX_FDFV_SOURCE CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/source/**.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/**.hpp)

if (${AX_BUILD_SHARED})
  add_library(axes_fdfv SHARED ${AX_FDFV_SOURCE})
else()
  add_library(axes_fdfv STATIC ${AX_FDFV_SOURCE})
endif()

target_include_directories(axes_fdfv
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(axes_fdfv
  PUBLIC
    axes
)

ax_set_warn_levels(axes_fdfv)
target_compile_options(axes_fdfv PRIVATE ${AX_CXX_FLAGS})
