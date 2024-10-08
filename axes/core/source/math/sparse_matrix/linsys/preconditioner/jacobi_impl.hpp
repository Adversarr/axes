#pragma once
#include "ax/core/buffer/buffer_view.hpp"
#include "ax/math/sparse_matrix/block_matrix.hpp"

namespace ax::math::details {
void jacobi_precond_precompute_blocked_cpu(RealBufferView dst, const RealBlockMatrix& mat);

void jacobi_precond_solve_cpu(RealBufferView dst, ConstRealBufferView rhs,
                              ConstRealBufferView inv_diag);

void jacobi_precond_precompute_blocked_gpu(RealBufferView dst, const RealBlockMatrix& mat);

void jacobi_precond_solve_gpu(RealBufferView dst, ConstRealBufferView rhs,
                              ConstRealBufferView inv_diag);

}  // namespace ax::math::details