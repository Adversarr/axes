find_package(Eigen3 CONFIG REQUIRED)
find_package(absl CONFIG REQUIRED)
find_package(OpenMP)
find_package(range-v3 CONFIG REQUIRED)
find_package(EnTT CONFIG REQUIRED)
find_package(libigl CONFIG REQUIRED)
find_package(blosc CONFIG REQUIRED)
find_package(OpenVDB CONFIG REQUIRED)
find_package(ZLIB REQUIRED)

# Library axes-core
file(GLOB_RECURSE AXES_CORE_SOURCE CONFIGURE_DEPENDS
${CMAKE_CURRENT_SOURCE_DIR}/source/**.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/**.hpp)

add_library(axes ${AXES_CORE_SOURCE})
target_include_directories(axes PRIVATE ${TINYGLTF_INCLUDE_DIRS})
target_include_directories(axes PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

set(ABSL_LIBRARIES
  absl::btree
  absl::check
  absl::flat_hash_map
  absl::inlined_vector
  absl::status
  absl::log
  absl::log_initialize
  absl::log_globals
  absl::log_flags
  absl::flags_parse
  absl::flags_usage
  absl::failure_signal_handler
  absl::statusor)

target_link_libraries(axes
  PUBLIC
  Eigen3::Eigen
  ${ABSL_LIBRARIES}
  EnTT::EnTT
  range-v3::range-v3
  OpenVDB::openvdb
  igl::igl_core
  blosc
  ZLIB::ZLIB
)

target_compile_options(axes PUBLIC -ftime-trace)

if(OpenMP_CXX_FOUND)
  target_link_libraries(axes PUBLIC OpenMP::OpenMP_CXX)
  message(STATUS "OpenMP found and link to axes.")
  target_compile_definitions(axes PRIVATE "AXES_HAS_OPENMP")
endif()


if(AXES_DEBUG_INTERNAL)
  target_compile_definitions(axes PUBLIC "AXES_DEBUG_INTERNAL")
endif()

axes_set_warn_levels(axes)
target_compile_definitions(axes PRIVATE AX_ASSET_DIR="${AX_ASSET_DIR}")

# Disable Exceptions
if(AXES_NO_EXCEPT)
  if(MSVC)
    target_compile_options(axes PRIVATE /EHsc)
  else()
    target_compile_options(axes PRIVATE -fno-exceptions)
  endif()
endif()

# PCH
target_precompile_headers(axes PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/axes/ax_pch.hpp)
