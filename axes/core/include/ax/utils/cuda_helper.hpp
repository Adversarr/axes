#pragma once
#include "ax/core/excepts.hpp"

#ifdef AX_HAS_CUDA

#  define AX_CUDA_CALL(expr) (expr)

#  define AX_CHECK_CUDA_ERRORS(val)                                        \
    do {                                                                   \
      cudaError_t err = (val);                                             \
      if (err != cudaSuccess) {                                            \
        AX_THROW_RUNTIME_ERROR("CUDA error {}: {}", cudaGetErrorName(err), \
                               cudaGetErrorString(err));                   \
      }                                                                    \
    } while (0)

#else

#  define AX_CUDA_CALL(expr) AX_THROW_RUNTIME_ERROR("CUDA is not enabled, try to call [" #expr "]")

#  define AX_CHECK_CUDA_ERRORS(expr) \
    AX_THROW_RUNTIME_ERROR("CUDA is not enabled, try to call [" #expr "]")

#endif

namespace ax::utils {

template <typename Integer>
Integer up_div(Integer a, Integer b) {
  static_assert(std::is_integral_v<Integer>, "Integer must be an integral type.");
  static_assert(std::is_unsigned_v<Integer>, "Integer must be an unsigned type.");
  return (a + b - 1) / b;
}

}  // namespace ax::utils