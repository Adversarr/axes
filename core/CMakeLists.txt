find_package(Eigen3 CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(cxxopts CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(Taskflow CONFIG REQUIRED)

# Library axes-core
file(GLOB_RECURSE AXES_CORE_SOURCE CONFIGURE_DEPENDS
     ${CMAKE_CURRENT_SOURCE_DIR}/source/**.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/include/**.hpp)
add_library(AxesCore ${AXES_CORE_SOURCE})

target_include_directories(AxesCore PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(
  AxesCore PUBLIC cxxopts::cxxopts Eigen3::Eigen nlohmann_json::nlohmann_json
                  spdlog::spdlog spdlog::spdlog_header_only Taskflow::Taskflow)

axes_set_warn_levels(AxesCore)

find_package(doctest CONFIG REQUIRED)
include(doctest)
file(GLOB_RECURSE ACORE_TEST_SRC "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp")
add_executable(axes_core_test ${ACORE_TEST_SRC})
target_link_libraries(axes_core_test PUBLIC AxesCore doctest::doctest)
