#include "ax/math/buffer_blas.hpp"

#include "ax/core/buffer/copy.hpp"
#include "ax/core/buffer/for_each.hpp"
#include "ax/core/entt.hpp"
#ifdef AX_HAS_BLAS
#  include <openblas/cblas.h>
#endif

#ifdef AX_HAS_CUDA
#  include <cublas_v2.h>
#endif

namespace ax::math::buffer_blas {
#ifdef AX_HAS_CUDA
namespace details {

struct CuBlasContext {
  CuBlasContext() {
    auto status = cublasCreate_v2(&handle_);
    AX_CHECK(status == CUBLAS_STATUS_SUCCESS,
             "CuBlasContext: failed to create cublas handle, status: {}",
             cublasGetStatusString(status));
  }

  ~CuBlasContext() {
    if (handle_) {
      cublasDestroy_v2(handle_);
    }
    handle_ = nullptr;
  }

  CuBlasContext(CuBlasContext const &) = delete;

  CuBlasContext(CuBlasContext &&other) {
    handle_ = other.handle_;
    other.handle_ = nullptr;
  }

  cublasHandle_t handle_;
};

cublasHandle_t ensure_cublas() {
  return ensure_resource<CuBlasContext>().handle_;
}
}  // namespace details
#endif

void do_copy_host(RealBufferView y, ConstRealBufferView x) {
#ifdef AX_HAS_BLAS
  size_t inc_x = x.Stride().X() / sizeof(Real);
  size_t inc_y = y.Stride().X() / sizeof(Real);
  size_t total = prod(x.Shape());
  cblas_dcopy(static_cast<blasint>(total), x.Data(), static_cast<blasint>(inc_x), y.Data(),
              static_cast<blasint>(inc_y));
#else
  ::ax::copy(y, x);
#endif
}

void do_copy_device(RealBufferView y, ConstRealBufferView x) {
#ifdef AX_HAS_CUDA
  auto handle = details::ensure_cublas();
  size_t total = prod(x.Shape());
  size_t inc_x = x.Stride().X() / sizeof(Real);
  size_t inc_y = y.Stride().X() / sizeof(Real);

  auto status = cublasDcopy_v2(handle, static_cast<int>(total), x.Data(), static_cast<int>(inc_x),
                               y.Data(), static_cast<int>(inc_y));
  AX_THROW_IF_NE(status, CUBLAS_STATUS_SUCCESS, "do_copy_device: failed to copy, status: {}",
                 cublasGetStatusString(status));
#else
  throw make_invalid_argument("do_copy_device: CUDA is not enabled.");
#endif
}

// computes y = x.
void copy(RealBufferView y, ConstRealBufferView x) {
  AX_CHECK(is_same_device(x, y), "copy: x and y should be on the same device.");
  if (x.Device() == BufferDevice::Host) {
    do_copy_host(y, x);
  } else {
    do_copy_device(y, x);
  }
}

void do_scal_host(Real alpha, RealBufferView x) {
#ifdef AX_HAS_BLAS
  size_t inc_x = x.Stride().X() / sizeof(Real);
  size_t total = prod(x.Shape());
  cblas_dscal(static_cast<blasint>(total), alpha, x.Data(), static_cast<blasint>(inc_x));
#else
  // TODO: Use Eigen Map.
  for_each(x, [alpha](Real &v) {
    v *= alpha;
  });
#endif
}

void do_scal_device(Real alpha, RealBufferView x) {
#ifdef AX_HAS_CUDA
  auto handle = details::ensure_cublas();
  size_t total = prod(x.Shape());
  size_t inc_x = x.Stride().X() / sizeof(Real);
  auto status
      = cublasDscal_v2(handle, static_cast<int>(total), &alpha, x.Data(), static_cast<int>(inc_x));
  AX_THROW_IF_NE(status, CUBLAS_STATUS_SUCCESS, "do_scal_device: failed to scale, status: {}",
                 cublasGetStatusString(status));
#else
  AX_CHECK(false, "do_scal_device: CUDA is not enabled.");
#endif
}

// computes x = alpha * x.
void scal(Real alpha, RealBufferView x) {
  if (!(x.IsContinuous(1))) {
    throw make_invalid_argument("scal: x must be continuous for y,z");
  }

  if (x.Device() == BufferDevice::Host) {
    do_scal_host(alpha, x);
  } else {
    do_scal_device(alpha, x);
  }
}

void do_swap_host(RealBufferView x, RealBufferView y) {
#ifdef AX_HAS_BLAS
  size_t inc_x = x.Stride().X() / sizeof(Real);
  size_t inc_y = y.Stride().X() / sizeof(Real);
  size_t total = prod(x.Shape());
  cblas_dswap(static_cast<blasint>(total), x.Data(), static_cast<blasint>(inc_x), y.Data(),
              static_cast<blasint>(inc_y));
#else
  for_each(std::tuple{x, y}, [](Real &a, Real &b) {
    std::swap(a, b);
  });
#endif
}

void do_swap_device(RealBufferView x, RealBufferView y) {
#ifdef AX_HAS_CUDA
  auto handle = details::ensure_cublas();
  size_t total = prod(x.Shape());
  size_t inc_x = x.Stride().X() / sizeof(Real);
  size_t inc_y = y.Stride().X() / sizeof(Real);
  auto status = cublasDswap_v2(handle, static_cast<int>(total), x.Data(), static_cast<int>(inc_x),
                               y.Data(), static_cast<int>(inc_y));
  AX_THROW_IF_NE(status, CUBLAS_STATUS_SUCCESS, "do_swap_device: failed to swap, status: {}",
                 cublasGetStatusString(status));
#else
  AX_CHECK(false, "do_swap_device: CUDA is not enabled.");
#endif
}

// swaps x and y.
void swap(RealBufferView x, RealBufferView y) {
  AX_CHECK(is_same_device(x, y), "swap: x and y should be on the same device.");
  if (!(x.IsContinuous(1) && y.IsContinuous(1))) {
    throw make_invalid_argument("swap: x and y must be continuous for y,z");
  }

  if (x.Device() == BufferDevice::Host) {
    do_swap_host(x, y);
  } else {
    do_swap_device(x, y);
  }
}

void do_axpy_host(Real alpha, ConstRealBufferView x, RealBufferView y) {
#ifdef AX_HAS_BLAS
  size_t inc_x = x.Stride().X() / sizeof(Real);
  size_t inc_y = y.Stride().X() / sizeof(Real);
  size_t total = prod(x.Shape());
  cblas_daxpy(static_cast<blasint>(total), alpha, x.Data(), static_cast<blasint>(inc_x), y.Data(),
              static_cast<blasint>(inc_y));
#else
  for_each(std::tuple{x, y}, [alpha](Real &a, Real &b) {
    b += alpha * a;
  });
#endif
}

void do_axpy_device(Real alpha, ConstRealBufferView x, RealBufferView y) {
#ifdef AX_HAS_CUDA
  auto handle = details::ensure_cublas();
  size_t total = prod(x.Shape());
  size_t inc_x = x.Stride().X() / sizeof(Real);
  size_t inc_y = y.Stride().X() / sizeof(Real);
  auto status = cublasDaxpy_v2(handle, static_cast<int>(total), &alpha, x.Data(),
                               static_cast<int>(inc_x), y.Data(), static_cast<int>(inc_y));
  AX_THROW_IF_NE(status, CUBLAS_STATUS_SUCCESS, "do_axpy_device: failed to axpy, status: {}",
                 cublasGetStatusString(status));
#else
  AX_CHECK(false, "do_axpy_device: CUDA is not enabled.");
#endif
}

// computes y = alpha * x + y.
void axpy(Real alpha, ConstRealBufferView x, RealBufferView y) {
  AX_CHECK(is_same_device(x, y), "axpy: x and y should be on the same device.");
  if (!(x.IsContinuous(1) && y.IsContinuous(1))) {
    throw make_invalid_argument("axpy: x and y must be continuous for y,z");
  }

  if (x.Device() == BufferDevice::Host) {
    do_axpy_host(alpha, x, y);
  } else {
    do_axpy_device(alpha, x, y);
  }
}

void do_dot_host(ConstRealBufferView x, ConstRealBufferView y, Real &result) {
#ifdef AX_HAS_BLAS
  size_t inc_x = x.Stride().X() / sizeof(Real);
  size_t inc_y = y.Stride().X() / sizeof(Real);
  size_t total = prod(x.Shape());
  result = cblas_ddot(static_cast<blasint>(total), x.Data(), static_cast<blasint>(inc_x), y.Data(),
                      static_cast<blasint>(inc_y));
#else
  result = 0;  // Optimize use openmp reduce.
  for_each(std::tuple{x, y}, [&result](Real a, Real b) {
    result += a * b;
  });
#endif
}

void do_dot_device(ConstRealBufferView x, ConstRealBufferView y, Real &result) {
#ifdef AX_HAS_CUDA
  auto handle = details::ensure_cublas();
  size_t total = prod(x.Shape());
  size_t inc_x = x.Stride().X() / sizeof(Real);
  size_t inc_y = y.Stride().X() / sizeof(Real);
  auto status = cublasDdot_v2(handle, static_cast<int>(total), x.Data(), static_cast<int>(inc_x),
                              y.Data(), static_cast<int>(inc_y), &result);
  AX_THROW_IF_NE(status, CUBLAS_STATUS_SUCCESS, "do_dot_device: failed to dot, status: {}",
                 cublasGetStatusString(status));
#else
  AX_CHECK(false, "do_dot_device: CUDA is not enabled.");
#endif
}

// computes dot product <x, y>
Real dot(ConstRealBufferView x, ConstRealBufferView y) {
  AX_CHECK(is_same_device(x, y), "dot: x and y should be on the same device.");
  Real result = 0;
  if (!(x.IsContinuous(1) && y.IsContinuous(1))) {
    throw make_invalid_argument("dot: x and y must be continuous for y,z");
  }

  if (x.Device() == BufferDevice::Host) {
    do_dot_host(x, y, result);
  } else {
    do_dot_device(x, y, result);
  }
  return result;
}

void do_nrm2_host(ConstRealBufferView x, Real &result) {
#ifdef AX_HAS_BLAS
  size_t inc_x = x.Stride().X() / sizeof(Real);
  size_t total = prod(x.Shape());
  result = cblas_dnrm2(static_cast<blasint>(total), x.Data(), static_cast<blasint>(inc_x));
#else
  result = 0;  // Optimize use openmp reduce.
  for_each(x, [&result](Real a) {
    result += a * a;
  });
  result = std::sqrt(result);
#endif
};

void do_nrm2_device(ConstRealBufferView x, Real &result) {
#ifdef AX_HAS_CUDA
  auto handle = details::ensure_cublas();
  size_t total = prod(x.Shape());
  size_t inc_x = x.Stride().X() / sizeof(Real);
  auto status
      = cublasDnrm2_v2(handle, static_cast<int>(total), x.Data(), static_cast<int>(inc_x), &result);
  AX_THROW_IF_NE(status, CUBLAS_STATUS_SUCCESS, "do_nrm2_device: failed to nrm2, status: {}",
                 cublasGetStatusString(status));
#else
  AX_CHECK(false, "do_nrm2_device: CUDA is not enabled.");
#endif
}

// computes the 2-norm of x.
Real norm(ConstRealBufferView x) {
  Real result = 0;
  if (!(x.IsContinuous(1))) {
    throw make_invalid_argument("norm: x must be continuous for y,z");
  }

  if (x.Device() == BufferDevice::Host) {
    do_nrm2_host(x, result);
  } else {
    do_nrm2_device(x, result);
  }
  return result;
}

void do_asum_host(ConstRealBufferView x, Real &result) {
#ifdef AX_HAS_BLAS
  size_t inc_x = x.Stride().X() / sizeof(Real);
  size_t total = prod(x.Shape());
  result = cblas_dasum(static_cast<blasint>(total), x.Data(), static_cast<blasint>(inc_x));
#else
  result = 0;  // Optimize use openmp reduce.
  for_each(x, [&result](Real a) {
    result += std::abs(a);
  });
#endif
}

void do_asum_device(ConstRealBufferView x, Real &result) {
#ifdef AX_HAS_CUDA
  auto handle = details::ensure_cublas();
  size_t total = prod(x.Shape());
  size_t inc_x = x.Stride().X() / sizeof(Real);
  auto status
      = cublasDasum_v2(handle, static_cast<int>(total), x.Data(), static_cast<int>(inc_x), &result);
  AX_THROW_IF_NE(status, CUBLAS_STATUS_SUCCESS, "do_asum_device: failed to asum, status: {}",
                 cublasGetStatusString(status));
#else
  AX_CHECK(false, "do_asum_device: CUDA is not enabled.");
#endif
}

// computes the 1-norm of x.
Real asum(ConstRealBufferView x) {
  Real result = 0;
  if (!(x.IsContinuous(1))) {
    throw make_invalid_argument("asum: x must be continuous for y,z");
  }

  if (x.Device() == BufferDevice::Host) {
    do_asum_host(x, result);
  } else {
    do_asum_device(x, result);
  }
  return result;
}

void do_amax_host(ConstRealBufferView x, Real &result) {
#ifdef AX_HAS_BLAS
  size_t inc_x = x.Stride().X() / sizeof(Real);
  size_t total = prod(x.Shape());
  result = cblas_damax(static_cast<blasint>(total), x.Data(), static_cast<blasint>(inc_x));
#else
  result = 0;  // Optimize use openmp reduce.
  for_each(x, [&result](Real a) {
    result = std::max(result, std::abs(a));
  });
#endif
}

// computes the max-norm of x.
Real amax(ConstRealBufferView x) {
  Real result = 0;
  if (!(x.IsContinuous(1))) {
    throw make_invalid_argument("amax: x must be continuous for y,z");
  }

  if (x.Device() == BufferDevice::Host) {
    do_amax_host(x, result);
  } else {
    // do_amax_device(x, result);
    throw make_invalid_argument("amax: not implemented for device buffer.");
  }
  return result;
}

}  // namespace ax::math::buffer_blas