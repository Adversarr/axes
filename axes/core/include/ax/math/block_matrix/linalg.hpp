// Implements some critical BLAS-style operations on buffers.
// The underlying implementation is depending on the compile options:
// The default, and always works(on Host) is Eigen.
// If you have:
//  AX_HAS_BLAS => use OpenBLAS for big buffers.
// for Device buffer.
//  AX_HAS_CUDA => for Device buffer.

#pragma once
#include "ax/core/buffer/buffer_view.hpp"

// Notice that, the stride of buffers are used to determine the BLAS parameter
// Most op are valid only when stride.y and stride.z are default. and stride.x are used to determine
// the INCX, or INCY parameter of BLAS operations.
namespace ax::math {

// the operation type for BLAS operations, we do not have complex numbers,
// so only transpose is supported.
AX_DEFINE_ENUM_CLASS(BlasOperation, None, Transpose);

///// LEVEL 1 /////

// computes y = x.
void copy(ConstRealBufferView x, RealBufferView y);
// computes x = alpha * x.
void scal(Real alpha, RealBufferView x);
// swaps x and y.
void swap(RealBufferView x, RealBufferView y);
// computes y = alpha * x + y.
void axpy(Real alpha, ConstRealBufferView x, RealBufferView y);
// computes dot product <x, y> into dst.
void dot(ConstRealBufferView x, ConstRealBufferView y, RealBufferView dst);
// computes the 2-norm of x.
Real norm(ConstRealBufferView x);
// computes the 1-norm of x.
Real asum(ConstRealBufferView x);
// computes the max-norm of x.
Real amax(ConstRealBufferView x);

///// LEVEL 2 /////

// computes y = alpha * A * x + beta * y.
void gemv(ConstRealBufferView mat_a, ConstRealBufferView vec_x, RealBufferView vec_y,
          Real alpha = 1.0, Real beta = 0.0, BlasOperation op = BlasOperation::None);

///// LEVEL 3 /////

// computes C = alpha * A * B + beta * C.
void gemm(ConstRealBufferView mat_a, ConstRealBufferView mat_b, RealBufferView mat_c,
          Real alpha = 1.0, Real beta = 0.0, BlasOperation op_a = BlasOperation::None,
          BlasOperation op_b = BlasOperation::None);

}  // namespace ax::math