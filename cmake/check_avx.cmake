include(CheckCXXSourceCompiles)

# Define the source code snippet for AVX2 detection
set(AVX2_CXX_CODE "#include <immintrin.h>
int main() {
#if __AVX2__
return 0;
#else
#error \"AVX2 is not supported\"
#endif
}")

# Define the source code snippet for AVX-512 detection
set(AVX512_CXX_CODE "#include <immintrin.h>
int main() {
#if __AVX512F__
return 0;
#else
#error \"AVX-512 is not supported\"
#endif
}")

# Check for AVX2 support
check_cxx_source_compiles("${AVX2_CXX_CODE}" AVX2_SUPPORTED)

if(AVX2_SUPPORTED)
  message("AVX2 is supported")
  ax_add_cxx_compiler_flag("-mavx2")
else()
  message("AVX2 is not supported")
endif()

# Check for AVX-512 support
check_cxx_source_compiles("${AVX512_CXX_CODE}" AVX512_SUPPORTED)

if(AVX512_SUPPORTED)
  message("AVX-512 is supported")
  ax_add_cxx_compiler_flag("-mavx512f")
else()
  message("AVX-512 is not supported")
endif()