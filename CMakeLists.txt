cmake_minimum_required(VERSION 3.23)

##############################################################################################
# Critical CMake settings.
##############################################################################################
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build." FORCE)
endif()

# guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(
    FATAL_ERROR
    "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt. "
  )
endif()

##############################################################################################
# Options for axes.
##############################################################################################
option(AX_INSTALL_SIGNAL_HANDLER "Enable Signal Handler from absl" ON)
option(AX_DEBUG_INTERNAL "Enable detailed debug asserts for internal implementation" ON)
option(AX_ENABLE_TEST "Enable test." ON)
option(AX_BUILD_SHARED "Build shared library" OFF)
option(AX_NO_EXCEPT "Enable noexcept, may lead to compile error for igl." OFF)
option(AX_ENABLE_TIME_TRACE "Enable time trace." OFF)
option(AX_ENABLE_AVX "Enable AVX." OFF)
option(AX_ENABLE_CUDA "Enable cuda." OFF)
option(AX_NO_WARNING "Disable all warnings." OFF)
option(AX_ENABLE_OPENMP "Enable OpenMP for automatic multi-threading in Eigen." ON)
set(AX_SDK_PATH "" CACHE PATH "Path to the SDK")

project(axes LANGUAGES C CXX)
include(cmake/utils.cmake)

# ===> Begin: Took from ZENO.
if (NOT DEFINED CMAKE_POSITION_INDEPENDENT_CODE)
    # Otherwise we can't link .so libs with .a libs
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()

if (WIN32)
    # get rid of 'min', 'max' macros; get back 'M_PI', 'M_E'
    add_definitions(-DNOMINMAX -D_USE_MATH_DEFINES)
endif()

if (AX_NO_WARNING)  # Use our customized warning flags
    set(AX_OLD_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    if (CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-all -Wno-cpp -Wno-deprecated-declarations -Wno-enum-compare -Wno-ignored-attributes -Wno-extra -Wreturn-type -Wmissing-declarations -Wnon-virtual-dtor -Wsuggest-override -Wconversion-null -Wuninitialized -Wno-format-security -Wnarrowing -Wno-maybe-uninitialized")
    endif()
    if (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
    endif()
else()
  if(MSVC)
    ax_add_cxx_compiler_flag("/W4")
  else()
    ax_add_cxx_compiler_flag("-Wall -Wextra -Wno-deprecated-declarations -Wno-enum-compare")
  endif()
endif()
# <=== End: Took from ZENO.

##############################################################################################
# Setup CUDA.
##############################################################################################

if (APPLE)
  if(AX_ENABLE_CUDA)
    message(STATUS "CUDA is not supported on macOS.")
    set(AX_ENABLE_CUDA OFF)
  endif()
endif()

if (AX_ENABLE_CUDA)
  include(CheckLanguage)
  check_language(CUDA)
  if (NOT CMAKE_CUDA_COMPILER)
    message(FATAL_ERROR "CUDA is enabled but no CUDA compiler found.")
  endif()

  find_package(CUDAToolkit REQUIRED)
  message(STATUS "CUDAToolkit_ROOT: ${CUDAToolkit_ROOT}")
  message(STATUS "CUDAToolkit_ROOT_DIR: ${CUDAToolkit_ROOT_DIR}")
  message(STATUS "CUDAToolkit_NVCC_EXECUTABLE: ${CUDAToolkit_NVCC_EXECUTABLE}")
  message(STATUS "CUDAToolkit_BIN_DIR: ${CUDAToolkit_BIN_DIR}")
  message(STATUS "CUDAToolkit_INCLUDE_DIRS: ${CUDAToolkit_INCLUDE_DIRS}")
  message(STATUS "CUDAToolkit_LIBRARY_DIR: ${CUDAToolkit_LIBRARY_DIR}")
  message(STATUS "CUDAToolkit_VERSION: ${CUDAToolkit_VERSION}")
  message(STATUS "CMAKE_CUDA_COMPILER: ${CMAKE_CUDA_COMPILER}")
  message(STATUS "CMAKE_CUDA_COMPILER_VERSION: ${CMAKE_CUDA_COMPILER_VERSION}")
  message(STATUS "CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES: ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}")
  enable_language(CUDA)
  set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)
  set(CMAKE_CUDA_STANDARD 20)
  set(CMAKE_CUDA_STANDARD_REQUIRED ON)
  set(CMAKE_CUDA_ARCHITECTURES 86)
  set(CMAKE_CUDA)
  list(APPEND AX_CUDA_FLAGS "--extended-lambda" "--expt-relaxed-constexpr")
endif()

##############################################################################################
# Setup lookup paths for all the dependencies.
##############################################################################################
if (EXISTS "${AX_SDK_PATH}")
  message(STATUS "AX_SDK_PATH: ${AX_SDK_PATH}")
else()
  set(AX_SDK_PATH "${CMAKE_CURRENT_LIST_DIR}/sdk")
  message(STATUS "AX_SDK_PATH is not set, using default path: ${AX_SDK_PATH}")
endif()
set(AX_SDK_PATH_WITH_BUILD_TYPE "${AX_SDK_PATH}/${CMAKE_BUILD_TYPE}")
if (NOT EXISTS "${AX_SDK_PATH_WITH_BUILD_TYPE}")
  set(AX_SDK_PATH_WITH_BUILD_TYPE "${AX_SDK_PATH}/RelWithDebInfo")
  message(STATUS "AX_SDK_PATH_WITH_BUILD_TYPE does not exist: ${AX_SDK_PATH_WITH_BUILD_TYPE}, use default=${AX_SDK_PATH_WITH_BUILD_TYPE}")
endif()
if (NOT EXISTS "${AX_SDK_PATH_WITH_BUILD_TYPE}")
  message(FATAL_ERROR "AX_SDK_PATH_WITH_BUILD_TYPE does not exist: ${AX_SDK_PATH_WITH_BUILD_TYPE}")
endif()

list(APPEND CMAKE_PREFIX_PATH "${AX_SDK_PATH_WITH_BUILD_TYPE}")
list(APPEND CMAKE_MODULE_PATH "${AX_SDK_PATH_WITH_BUILD_TYPE}/lib/cmake/OpenVDB")
set(libigl_ROOT "${CMAKE_PREFIX_PATH}/lib/cmake/igl")
message(STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")
message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_BUILD_TYPE}/bin/)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_BUILD_TYPE}/lib/)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_BUILD_TYPE}/lib/)

if (WIN32)
  set(AX_DLL_DIR "${AX_SDK_PATH_WITH_BUILD_TYPE}/bin")
  set(AX_DLL_EXT "dll")
else()
  set(AX_DLL_DIR "${AX_SDK_PATH_WITH_BUILD_TYPE}/lib")
  if (APPLE)
    set(AX_DLL_EXT "dylib")
  else()
    set(AX_DLL_EXT "so")
  endif()
endif()

# Copy the SDK/bin to runtime path:
file(GLOB SDK_BIN_FILES RELATIVE "${AX_DLL_DIR}" "${AX_DLL_DIR}/*.${AX_DLL_EXT}")
foreach(SDK_BIN_FILE ${SDK_BIN_FILES})
  # If not exist, do the copy
  set(SDK_BIN_PATH "${AX_DLL_DIR}/${SDK_BIN_FILE}")
  if (NOT EXISTS ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${SDK_BIN_FILE})
    message(STATUS "==> ${SDK_BIN_FILE} does not exsit, copying...")
    file(COPY ${SDK_BIN_PATH} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
  endif()
endforeach()


##############################################################################################
# Some common compiler flags.
##############################################################################################
# MSVC is special.
if (MSVC)
  ax_add_cxx_compiler_flag("/bigobj")
endif()

# Further optimize for SIMD.
if (AX_ENABLE_AVX)
  if(NOT MSVC)
    ax_add_cxx_compiler_flag("-march=native")
    include(cmake/check_avx.cmake)
  else()
    # ax_add_cxx_compiler_flag("/arch:AVX") Eigen still have bug with AVX2
  endif()
endif()


# Disable Exceptions
if(AX_NO_EXCEPT)
  if(MSVC)
    ax_add_cxx_compiler_flag("/EHsc")
  else()
    ax_add_cxx_compiler_flag("-fno-exceptions")
  endif()
endif()

# Time trace
if(AX_ENABLE_TIME_TRACE AND NOT MSVC)
  ax_add_cxx_compiler_flag("-ftime-trace")
endif()

# Default Asset Directory.
set(AX_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

##############################################################################################
# Now, declare the library and executables.
##############################################################################################
add_subdirectory(axes)
add_subdirectory(examples)

if (${AX_ENABLE_TEST})
  enable_testing()
  add_subdirectory(test)
endif()


