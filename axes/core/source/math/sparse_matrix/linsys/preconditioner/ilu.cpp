#include "ax/math/sparse_matrix/linsys/preconditioner/ilu.hpp"
#include "ax/core/buffer/eigen_support.hpp"
#include "ax/math/sparse_matrix/csr.hpp"
#include <Eigen/IterativeLinearSolvers>


namespace ax::math {

// currently ILU preconditioner just support CPU.
struct GeneralSparsePreconditioner_ILU::Impl {

  Impl() {
    ilut_.setDroptol(1e-3);
  }

  ~Impl() = default;

  void Solve(ConstRealBufferView b, RealBufferView x) const {
    size_t x_size = prod(x.Shape()), b_size = prod(b.Shape());
    if (x_size != b_size) {
      AX_THROW_INVALID_ARGUMENT("The size of x and b should be the same. x={}, b={}", x_size, b_size);
    }
    auto x_mapped = view_as_matrix_full<RealVectorX>(x.Reshaped({x_size, 1}));
    auto b_mapped = view_as_matrix_full<const RealVectorX>(b.Reshaped({b_size, 1}));
    x_mapped = ilut_.solve(b_mapped);
    if (ilut_.info() != Eigen::Success) {
      AX_THROW_RUNTIME_ERROR("ILU preconditioner solve failed: {}",
                              to_string(ilut_.info()));
    }
  }

  void Compute(ConstRealSparseMatrixPtr mat) {
    auto csr = mat->ToCSR();
    auto spm = csr->MapToEigen();
    ilut_.compute(spm);
    if (ilut_.info() != Eigen::Success) {
      AX_THROW_RUNTIME_ERROR("Failed to compute ILU preconditioner: {}",
                              to_string(ilut_.info()));
    }
  }
  Eigen::IncompleteLUT<Real> ilut_;
};

GeneralSparsePreconditioner_ILU::GeneralSparsePreconditioner_ILU() = default;
GeneralSparsePreconditioner_ILU::~GeneralSparsePreconditioner_ILU() = default;

void GeneralSparsePreconditioner_ILU::Solve(ConstRealBufferView b, RealBufferView x) const {
  AX_CHECK(impl_, "The preconditioner is not initialized.");
  impl_->Solve(b, x);
}

void GeneralSparsePreconditioner_ILU::AnalyzePattern() {
  impl_ = std::make_unique<Impl>();
}

void GeneralSparsePreconditioner_ILU::Factorize() {
  AX_CHECK(impl_, "The preconditioner is not initialized.");
  AX_THROW_IF(mat_->Device() == BufferDevice::Device, "ILU preconditioner does not support GPU.");
  impl_->Compute(mat_);
}

}  // namespace ax::math