find_package(Eigen3 CONFIG REQUIRED)
find_package(absl CONFIG REQUIRED)
find_package(OpenMP)
find_package(range-v3 CONFIG REQUIRED)
find_package(EnTT CONFIG REQUIRED)
find_package(OpenVDB REQUIRED)

message(STATUS "==> Eigen3: ${Eigen3_DIR}")
message(STATUS "==> absl: ${absl_DIR}")
message(STATUS "==> EnTT: ${EnTT_DIR}")
message(STATUS "==> range-v3: ${range-v3_DIR}")
message(STATUS "==> libigl: ${libigl_DIR}")
message(STATUS "==> OpenVDB: ${OpenVDB_INCLUDE_DIR}")
message(STATUS "==> zlib: ${zlib_INCLUDEDIR}")

# Library axes-core
file(GLOB_RECURSE AX_CORE_SOURCE CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/source/**.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/**.hpp)

if (${AX_BUILD_SHARED})
  add_library(axes SHARED ${AX_CORE_SOURCE})
else()
  add_library(axes STATIC ${AX_CORE_SOURCE})
endif()
target_include_directories(axes
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

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
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(axes PUBLIC OpenMP::OpenMP_CXX)
  message(STATUS "OpenMP found and link to axes.")
  target_compile_definitions(axes PRIVATE "AX_HAS_OPENMP")
endif()

ax_set_warn_levels(axes)
target_compile_definitions(axes PRIVATE AX_ASSET_DIR="${AX_ASSET_DIR}")

# Disable Exceptions
if(AX_NO_EXCEPT)
  if(MSVC)
    target_compile_options(axes PRIVATE /EHsc)
  else()
    target_compile_options(axes PRIVATE -fno-exceptions)
  endif()
endif()

target_compile_options(axes PRIVATE ${AX_CXX_FLAGS})
