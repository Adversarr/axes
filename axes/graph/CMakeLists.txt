# Library axes-core
find_package(Boost CONFIG REQUIRED COMPONENTS json graph)

file(GLOB_RECURSE AX_GRAPH_SOURCE CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/source/**.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/**.hpp)

if (${AX_BUILD_SHARED})
  add_library(axes_graph SHARED ${AX_GRAPH_SOURCE})
else()
  add_library(axes_graph STATIC ${AX_GRAPH_SOURCE})
endif()

target_include_directories(axes_graph
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(axes_graph
  PUBLIC
    axes_gl
    Boost::json
    Boost::graph
)

ax_target_apply_flag(axes_graph PRIVATE)
