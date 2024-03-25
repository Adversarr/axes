cmake_minimum_required(VERSION 3.17)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(AXES_INSTALL_SIGNAL_HANDLER "Enable Signal Handler from absl" ON)
option(AXES_DEBUG_INTERNAL "Enable detailed debug asserts for internal implementation" ON)
option(AXES_ENABLE_TEST "Enable test." ON)
option(AXES_BUILD_SHARED "Build shared library" OFF)
option(AXES_NO_EXCEPT "Enable noexcept, may lead to compile error for igl." OFF)
option(AXES_ENABLE_TIME_TRACE "Enable time trace." OFF)

# Remove this block after bumping CMake to v3.21.0 PROJECT_IS_TOP_LEVEL is
# defined then by default
if(CMAKE_VERSION VERSION_LESS 3.21.0)
  if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(PROJECT_IS_TOP_LEVEL TRUE)
  else()
    set(PROJECT_IS_TOP_LEVEL FALSE)
  endif()
endif()

# guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(
    FATAL_ERROR
    "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt. "
  )
endif()

# guard against bad build-type strings
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

project(axes)
include(cmake/utils.cmake)

if (MSVC)
  axes_add_cxx_compiler_flag("/bigobj")
  axes_add_cxx_compiler_flag("/Zc:__cplusplus")
endif()

# Further optimize for SIMD.
if(NOT MSVC)
  axes_add_cxx_compiler_flag("-march=native")
else()
  include(cmake/check_avx.cmake)
endif()

# Time trace
if(${AXES_ENABLE_TIME_TRACE} AND NOT MSVC)
  axes_add_cxx_compiler_flag("-ftime-trace")
endif()

# Default Asset Directory.
set(AX_ASSET_DIR "${CMAKE_CURRENT_SOURCE_DIR}/asset")


add_subdirectory(axes)
add_subdirectory(axes-gl)

if (AXES_ENABLE_TEST)
  enable_testing()
  add_subdirectory(test)
endif()

add_subdirectory(examples)

