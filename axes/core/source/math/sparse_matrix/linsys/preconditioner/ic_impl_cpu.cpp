#include "ax/core/buffer/eigen_support.hpp"
#include "ic_impl.hpp"

namespace ax::math {

void ImplIcCpu::AnalyzePattern() {
  AX_EXPECTS(mat_);
  auto device = mat_->Device();
  AX_EXPECTS(device == BufferDevice::Host);
  mat_eigen_ = mat_->ToSparseMatrix();

  ic_.analyzePattern(mat_eigen_);

  if (ic_.info() != Eigen::Success) {
    AX_THROW_RUNTIME_ERROR("IC analyze pattern failed: {}", to_string(ic_.info()));
  }
}

void ImplIcCpu::Factorize() {
  AX_EXPECTS(mat_);
  auto device = mat_->Device();
  AX_EXPECTS(device == BufferDevice::Host);
  mat_eigen_ = mat_->ToSparseMatrix();

  // Although we changed the value, but we assume a same sparse pattern for factorization
  ic_.compute(mat_eigen_);

  if (ic_.info() != Eigen::Success) {
    AX_THROW_RUNTIME_ERROR("IC factorization failed: {}", to_string(ic_.info()));
  }
}

void ImplIcCpu::Solve(ConstRealBufferView b, RealBufferView x) const {
  auto vb = view_as_matrix_full<const math::RealMatrixX>(b);
  auto vx = view_as_matrix_full<math::RealMatrixX>(x);

  vx.noalias() = ic_.solve(vb);
}

}  // namespace ax::math