#include "ic_impl.hpp"

namespace ax::math {

GeneralSparsePreconditioner_IncompleteCholesky::GeneralSparsePreconditioner_IncompleteCholesky()
    = default;

GeneralSparsePreconditioner_IncompleteCholesky::~
GeneralSparsePreconditioner_IncompleteCholesky() noexcept
    = default;

void GeneralSparsePreconditioner_IncompleteCholesky::AnalyzePattern() {
  AX_THROW_IF_NULLPTR(mat_, "mat not set.");
  // TODO: If you support other kind of matrix, you need to add them here.

  auto device = mat_->Device();
  if (!pimpl_ || pimpl_->Device() != device) {
    if (device == BufferDevice::Host) {
      pimpl_ = std::make_unique<ImplIcCpu>(mat_);
    } else {
      if (mat_->BlockSize() > 1) {
        std::once_flag of;
        std::call_once(of, []() {
          AX_WARN(
              "BSR matrix is not naively supported. AnalyzePattern will do noting and Factorize "
              "will use CSR format.");
        });
      } else {
        pimpl_ = std::make_unique<ImplIcCsrGpu>(mat_);
      }
    }
  }

  pimpl_->AnalyzePattern();
}

void GeneralSparsePreconditioner_IncompleteCholesky::Factorize() {
  if (!pimpl_ && mat_->Device() == BufferDevice::Device && mat_->BlockSize() > 1) {
    // BSR branch.
    pimpl_ = std::make_unique<ImplIcCsrGpu>(mat_->ToCSR());
  }

  AX_THROW_IF_NULLPTR(mat_, "mat not set.");
  AX_THROW_IF_NULLPTR(pimpl_, "Analyze pattern first.");

  pimpl_->Factorize();
}

void GeneralSparsePreconditioner_IncompleteCholesky::Solve(ConstRealBufferView b,
                                                           RealBufferView x) const {
  AX_THROW_IF_NULLPTR(pimpl_, "Analyze pattern first.");

  auto expect_shape = Dim3(mat_->Rows());

  if (is_2d(b.Shape())) {
    if (prod(b.Shape()) != mat_->Rows()) {
      AX_THROW_INVALID_ARGUMENT("b shape mismatch with mat.");
    }
    b = b.Reshaped(expect_shape);
  }

  if (is_2d(x.Shape())) {
    if (prod(x.Shape()) != mat_->Rows()) {
      AX_THROW_INVALID_ARGUMENT("x shape mismatch with mat.");
    }
    x = x.Reshaped(expect_shape);
  }

  AX_THROW_IF_NE(b.Shape(), expect_shape, "b shape mismatch with mat. expect {} got {}",
                 expect_shape, b.Shape());
  AX_THROW_IF_NE(x.Shape(), expect_shape, "x shape mismatch with mat. expect {} got {}",
                 expect_shape, x.Shape());

  pimpl_->Solve(b, x);
}

GeneralPreconditionerKind GeneralSparsePreconditioner_IncompleteCholesky::GetKind() const {
  return GeneralPreconditionerKind::IncompleteCholesky;
}

}  // namespace ax::math