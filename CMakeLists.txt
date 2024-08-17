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
option(AX_ENABLE_TEST "Enable test." ON)
option(AX_BUILD_SHARED "Build shared library" OFF)
option(AX_ENABLE_TIME_TRACE "Enable time trace." OFF)
option(AX_ENABLE_CUDA "Enable cuda." OFF)
option(AX_NO_WARNING "Disable all warnings." OFF)
option(AX_ENABLE_OPENMP "Enable OpenMP for automatic multi-threading in Eigen." ON)
option(AX_ENABLE_PYBIND "Enable pybind11." OFF)
option(AX_ENABLE_AVX "Enable AVX." OFF)
option(AX_ENABLE_FASTMATH "Enable fast math." OFF)
option(AX_ENABLE_MSAN "Enable Memory San." OFF)

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

if (NOT AX_NO_WARNING)  # Use our customized warning flags
  if(MSVC)
    ax_add_cxx_compiler_flag("/W4")
  else()
    # ax_add_cxx_compiler_flag("-Wall -Wextra -Wno-deprecated-declarations -Wno-enum-compare")
    ax_add_cxx_compiler_flag("-Wall")
    ax_add_cxx_compiler_flag("-Wextra")
    # ax_add_cxx_compiler_flag("-Weffc++") # TODO: This is too noisy.
    ax_add_cxx_compiler_flag("-Wuninitialized")
    ax_add_cxx_compiler_flag("-Wreturn-type")
    ax_add_cxx_compiler_flag("-Wconversion")
    ax_add_cxx_compiler_flag("-Wsign-compare")
    ax_add_cxx_compiler_flag("-Wunused-result")
    ax_add_cxx_compiler_flag("-Wsuggest-override")
    ax_add_cxx_compiler_flag("-Wzero-as-null-pointer-constant")
    ax_add_cxx_compiler_flag("-Wmissing-declarations")
    ax_add_cxx_compiler_flag("-Wold-style-cast")
    ax_add_cxx_compiler_flag("-Wnon-virtual-dtor")
  endif()
endif()

# <=== End: Took from ZENO.

# WARN: May not work.
if(AX_ENABLE_AVX)
    ax_add_cxx_compiler_flag_host_only("-march=native")
endif()

if (AX_ENABLE_FASTMATH)
  if (MSVC)
    ax_add_cxx_compiler_flag("/fp:fast")
  else()
    ax_add_cxx_compiler_flag("-ffast-math")
  endif()

  message(STATUS "Enable fast math.")
endif()

if(AX_ENABLE_MSAN)
  if (MSVC)
    message(FATAL_ERROR "MSAN is not supported on Windows.")
  else()
    ax_add_cxx_compiler_flag("-fsanitize=memory")
    ax_add_cxx_compiler_flag("-fno-omit-frame-pointer")
    ax_add_cxx_compiler_flag("-fsanitize-memory-track-origins=2")
    # ax_add_cxx_linker_flag("-fsanitize=memory")
  endif()
endif()

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
  set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)
  set(CMAKE_CUDA_STANDARD 20)
  set(CMAKE_CUDA_STANDARD_REQUIRED ON)
  set(CMAKE_CUDA_ARCHITECTURES 86)
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
  list(APPEND AX_CUDA_FLAGS "--extended-lambda" "--expt-relaxed-constexpr" "-rdc=true" "-c")
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

list(PREPEND CMAKE_PREFIX_PATH "${AX_SDK_PATH_WITH_BUILD_TYPE}")
list(PREPEND CMAKE_MODULE_PATH "${AX_SDK_PATH_WITH_BUILD_TYPE}/lib/cmake/OpenVDB")
set(libigl_ROOT "${CMAKE_PREFIX_PATH}/lib/cmake/igl")
message(STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")
message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_BUILD_TYPE}/bin/)
# if not set output directory:
if (NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_BUILD_TYPE}/lib/)
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_BUILD_TYPE}/lib/)
endif()

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
    message(STATUS "==> ${SDK_BIN_FILE} does not exsit, copying... ${SDK_BIN_PATH}->${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${SDK_BIN_FILE}")
    file(COPY ${SDK_BIN_PATH} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY} FOLLOW_SYMLINK_CHAIN)
  endif()
endforeach()


##############################################################################################
# Some common compiler flags.
##############################################################################################
# MSVC is special.
if (MSVC)
  ax_add_cxx_compiler_flag("/bigobj")
  ax_add_cxx_compiler_flag("/Zc:preprocessor")
endif()

# Time trace
if(AX_ENABLE_TIME_TRACE AND NOT MSVC)
  ax_add_cxx_compiler_flag("-ftime-trace")
endif()

# Default Asset Directory.
set(AX_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
# Disable LTO:
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION OFF)

##############################################################################################
# Now, declare the library and executables.
##############################################################################################
add_subdirectory(axes)
add_subdirectory(examples)

if (${AX_ENABLE_TEST})
  enable_testing()
  add_subdirectory(test)
endif()

if (${AX_ENABLE_PYBIND})
  add_subdirectory(binding)
endif()

