file(GLOB_RECURSE AXES_CORE_SOURCE CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/source/**.cpp)

add_library(AxesCore ${AXES_CORE_SOURCE})

target_include_directories(AxesCore PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
