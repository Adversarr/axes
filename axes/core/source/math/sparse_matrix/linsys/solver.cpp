#include "ax/math/sparse_matrix/linsys/solver.hpp"

#include "ax/core/gsl.hpp"
#include "ax/math/sparse_matrix/linsys/preconditioner.hpp"
#include "ax/math/sparse_matrix/linsys/solver/cg.hpp"
#include "ax/math/sparse_matrix/linsys/solver/downcast.hpp"

namespace ax::math {

void GeneralSparseSolverBase::SetProblem(ConstRealSparseMatrixPtr mat) {
  mat_ = std::move(mat);
}

void GeneralSparseSolverBase::AnalyzePattern() {
  if (preconditioner_) {
    preconditioner_->SetProblem(mat_);
    preconditioner_->AnalyzePattern();
  }
}

void GeneralSparseSolverBase::Factorize() {
  if (preconditioner_) {
    preconditioner_->Factorize();
  }
}

void GeneralSparseSolverBase::Compute() {
  AnalyzePattern();
  Factorize();
}

std::unique_ptr<GeneralSparseSolverBase> GeneralSparseSolverBase::Create(
    GeneralSparseSolverKind kind) {
  switch (kind) {
    case GeneralSparseSolverKind::ConjugateGradient:
      return std::make_unique<GeneralSparseSolver_ConjugateGradient>();
    case GeneralSparseSolverKind::Downcast:
      return std::make_unique<GeneralSparseSolver_Downcast>();
    default:
      AX_THROW_INVALID_ARGUMENT("Unknown solver kind");
  }
  AX_UNREACHABLE();
}

utils::Options GeneralSparseSolverBase::GetOptions() const {
  auto opt = Tunable::GetOptions();
  opt["max_iteration"] = max_iteration_;
  opt["tolerance"] = tolerance_;
  if (preconditioner_) {
    opt["preconditioner_kind"] = utils::reflect_name(preconditioner_->GetKind()).value_or("???");
    opt["preconditioner_opt"] = preconditioner_->GetOptions();
  }
  return opt;
}

void GeneralSparseSolverBase::SetOptions(utils::Options const& option) {
  AX_SYNC_OPT(option, size_t, max_iteration);
  AX_SYNC_OPT_IF(option, Real, tolerance) {
    if (tolerance_ <= 0) {
      AX_THROW_INVALID_ARGUMENT("tolerance must greater than zero. got {:12.6e}", tolerance_);
    }
  }

  utils::extract_and_create<GeneralPreconditionerKind, GeneralSparsePreconditionerBase>(
      option, "preconditioner_kind", preconditioner_);

  if (auto it = option.find("precondioner_opt");
      it != option.end()) {
    if (!preconditioner_) {
      AX_WARN("preconditioner is null, but precondioner_opt is set");
    }

    preconditioner_->SetOptions(it->value().as_object());
  }
}

}  // namespace ax::math