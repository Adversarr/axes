find_package(Eigen3 CONFIG REQUIRED)
# find_package(absl CONFIG REQUIRED)
find_package(range-v3 CONFIG REQUIRED)
find_package(EnTT CONFIG REQUIRED)
find_package(Blosc REQUIRED)
find_package(Boost REQUIRED COMPONENTS json)
find_package(ZLIB REQUIRED)
find_package(Boost REQUIRED COMPONENTS json)
find_package(OpenVDB REQUIRED)
find_package(CHOLMOD REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(cxxopts REQUIRED)

if (AX_ENABLE_OPENMP)
    find_package(OpenMP)
endif()

message(STATUS "==> Eigen3: ${Eigen3_DIR}")
# message(STATUS "==> absl: ${absl_DIR}")
message(STATUS "==> EnTT: ${EnTT_DIR}")
message(STATUS "==> Blosc: ${Blosc_INCLUDE_DIR}")
message(STATUS "==> range-v3: ${range-v3_DIR}")
message(STATUS "==> libigl: ${libigl_DIR}")
message(STATUS "==> OpenVDB: ${OpenVDB_INCLUDE_DIR}")
message(STATUS "==> zlib: ${ZLIB_INCLUDE_DIR}")

# Library axes-core
file(GLOB_RECURSE AX_CORE_SOURCE CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/source/**.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/**.hpp)

if (${AX_HAS_CUDA})
  file(GLOB_RECURSE AX_CORE_CUDA_SOURCE CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/source/**.cu
    ${CMAKE_CURRENT_SOURCE_DIR}/source/**.cuh)
  list(APPEND AX_CORE_SOURCE ${AX_CORE_CUDA_SOURCE})
endif()

if (${AX_BUILD_SHARED})
  add_library(axes_core SHARED ${AX_CORE_SOURCE})
else()
  add_library(axes_core STATIC ${AX_CORE_SOURCE})
endif()
target_include_directories(axes_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(axes_core
  PUBLIC
    Eigen3::Eigen
    # ${ABSL_LIBRARIES}
    cxxopts::cxxopts
    EnTT::EnTT
    range-v3::range-v3
    OpenVDB::openvdb
    Boost::json
    Blosc::blosc
    ZLIB::ZLIB
    Boost::json
    SuiteSparse::CHOLMOD
    spdlog::spdlog_header_only
    fmt::fmt-header-only
#    Backward::Interface
)

if (AX_ENABLE_CUDA)
  target_link_libraries(axes_core PUBLIC CUDA::cudart)
  target_compile_definitions(axes_core PUBLIC "AX_HAS_CUDA")
endif()

if (AX_ENABLE_OPENMP)
  if (OpenMP_CXX_FOUND)
    target_link_libraries(axes_core PUBLIC OpenMP::OpenMP_CXX)
    target_compile_definitions(axes_core PUBLIC "AX_HAS_OPENMP")
  endif()
endif()

target_compile_definitions(axes_core PRIVATE AX_ROOT_DIR="${AX_ROOT_DIR}")
ax_target_apply_flag(axes_core PRIVATE)
include(FindPkgConfig)
pkg_check_modules(libdw QUIET libdw)
if (libdw_FOUND)
  target_link_libraries(axes_core PUBLIC ${libdw_LIBRARIES})
  target_include_directories(axes_core PUBLIC ${libdw_INCLUDE_DIRS})
  target_compile_definitions(axes_core PRIVATE "AX_HAS_LIBDW")

  message(STATUS "libdw found, enabling advanced stacktrace")
endif()