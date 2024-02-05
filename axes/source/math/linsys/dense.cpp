#include "axes/math/linsys/dense.hpp"

#include "axes/core/echo.hpp"
#include "axes/utils/status.hpp"
namespace ax::math {

utils::uptr<DenseSolver> DenseSolver::Create(DenseSolverKind kind) {
  switch (kind) {
    case DenseSolverKind::kLDLT:
      return std::make_unique<DenseSolver_LDLT>();
    case DenseSolverKind::kLLT:
      return std::make_unique<DenseSolver_LLT>();
    case DenseSolverKind::kPartialPivLU:
      return std::make_unique<DenseSolver_PartialPivLU>();
    case DenseSolverKind::kFullPivLU:
      return std::make_unique<DenseSolver_FullPivLU>();
    case DenseSolverKind::kHouseholderQR:
      return std::make_unique<DenseSolver_HouseholderQR>();
    case DenseSolverKind::kColPivHouseholderQR:
      return std::make_unique<DenseSolver_ColPivHouseholderQR>();
    case DenseSolverKind::kFullPivHouseHolderQR:
      return std::make_unique<DenseSolver_FullPivHouseHolderQR>();
    case DenseSolverKind::kCompleteOrthognalDecomposition:
      return std::make_unique<DenseSolver_CompleteOrthognalDecomposition>();
    case DenseSolverKind::kJacobiSVD:
      return std::make_unique<DenseSolver_JacobiSVD>();
    case DenseSolverKind::kBDCSVD:
      return std::make_unique<DenseSolver_BDCSVD>();
    default:
      return nullptr;
  }
}

LinsysSolveResult DenseSolver_LLT::Solve(vecxr const& b, vecxr const&, utils::Opt const& options) {
  if (!(impl_.info() == Eigen::Success)) {
    return utils::FailedPreconditionError("The factorization has not been computed.");
  }
  vecxr x = impl_.solve(b);
  return LinsysSolveResultImpl{std::move(x)};
}

Status DenseSolver_LLT::Analyse(problem_t const& problem, utils::Opt const& options) {
  impl_.compute(problem.A_);
  if (!(impl_.info() == Eigen::Success)) {
    return utils::FailedPreconditionError("The factorization has not been computed.");
  }
  AX_RETURN_OK();
}

LinsysSolveResult DenseSolver_LDLT::Solve(vecxr const& b, vecxr const&, utils::Opt const& options) {
  if (!(impl_.info() == Eigen::Success)) {
    return utils::FailedPreconditionError("The factorization has not been computed.");
  }
  vecxr x = impl_.solve(b);
  return LinsysSolveResultImpl{std::move(x)};
}

Status DenseSolver_LDLT::Analyse(problem_t const& problem, utils::Opt const& options) {
  impl_.compute(problem.A_);
  if (!(impl_.info() == Eigen::Success)) {
    return utils::FailedPreconditionError("The factorization has not been computed.");
  }
  AX_RETURN_OK();
}

LinsysSolveResult DenseSolver_PartialPivLU::Solve(vecxr const& b, vecxr const&, utils::Opt const& options) {
  static bool logged = false;
  if (!logged) {
    LOG(WARNING) << "This method always your matrix is invertible. If you are not sure, use FullPivLU instead.";
    logged = true;
  }
  vecxr x = impl_.solve(b);
  return LinsysSolveResultImpl{std::move(x)};
}

Status DenseSolver_PartialPivLU::Analyse(problem_t const& problem, utils::Opt const& options) {
  impl_.compute(problem.A_);
  static bool logged = false;
  if (!logged) {
    LOG(WARNING) << "This method always your matrix is invertible. If you are not sure, use FullPivLU instead.";
    logged = true;
  }
  AX_RETURN_OK();
}

LinsysSolveResult DenseSolver_FullPivLU::Solve(vecxr const& b, vecxr const&, utils::Opt const& options) {
  if (!(impl_.isInjective())) {
    return utils::FailedPreconditionError("The factorization has not been computed.");
  }
  vecxr x = impl_.solve(b);
  return LinsysSolveResultImpl{std::move(x)};
}

Status DenseSolver_FullPivLU::Analyse(problem_t const& problem, utils::Opt const& options) {
  impl_.compute(problem.A_);
  if (!impl_.isInjective()) {
    return utils::FailedPreconditionError("The factorization has not been computed.");
  }
  AX_RETURN_OK();
}

LinsysSolveResult DenseSolver_HouseholderQR::Solve(vecxr const& b, vecxr const&, utils::Opt const& options) {
  if (!(impl_.isInjective())) {
    return utils::FailedPreconditionError("The factorization has not been computed.");
  }
  vecxr x = impl_.solve(b);
  return LinsysSolveResultImpl{std::move(x)};
}

Status DenseSolver_HouseholderQR::Analyse(problem_t const& problem, utils::Opt const& options) {
  impl_.compute(problem.A_);
  if (!(impl_.isInjective())) {
    return utils::FailedPreconditionError("The factorization has not been computed.");
  }
  AX_RETURN_OK();
}

LinsysSolveResult DenseSolver_ColPivHouseholderQR::Solve(vecxr const& b, vecxr const&, utils::Opt const& options) {
  if (!(impl_.isInjective())) {
    return utils::FailedPreconditionError("The factorization has not been computed.");
  }
  vecxr x = impl_.solve(b);
  return LinsysSolveResultImpl{std::move(x)};
}

Status DenseSolver_ColPivHouseholderQR::Analyse(problem_t const& problem, utils::Opt const& options) {
  impl_.compute(problem.A_);
  if (!(impl_.isInjective())) {
    return utils::FailedPreconditionError("The factorization has not been computed.");
  }
  AX_RETURN_OK();
}

LinsysSolveResult DenseSolver_FullPivHouseHolderQR::Solve(vecxr const& b, vecxr const&, utils::Opt const& options) {
  if (!(impl_.isInjective())) {
    return utils::FailedPreconditionError("The factorization has not been computed.");
  }
  vecxr x = impl_.solve(b);
  return LinsysSolveResultImpl{std::move(x)};
}

Status DenseSolver_FullPivHouseHolderQR::Analyse(problem_t const& problem, utils::Opt const& options) {
  impl_.compute(problem.A_);
  if (!(impl_.isInjective())) {
    return utils::FailedPreconditionError("The factorization has not been computed.");
  }
  AX_RETURN_OK();
}

LinsysSolveResult DenseSolver_CompleteOrthognalDecomposition::Solve(vecxr const& b, vecxr const&,
                                                                    utils::Opt const& options) {
  if (!(impl_.isInjective())) {
    return utils::FailedPreconditionError("The factorization has not been computed.");
  }
  vecxr x = impl_.solve(b);
  return LinsysSolveResultImpl{std::move(x)};
}

Status DenseSolver_CompleteOrthognalDecomposition::Analyse(problem_t const& problem, utils::Opt const& options) {
  impl_.compute(problem.A_);
  if (!(impl_.isInjective())) {
    return utils::FailedPreconditionError("The factorization has not been computed.");
  }
  AX_RETURN_OK();
}

LinsysSolveResult DenseSolver_JacobiSVD::Solve(vecxr const& b, vecxr const&, utils::Opt const& options) {
  vecxr x = impl_.solve(b);
  return LinsysSolveResultImpl{std::move(x)};
}

Status DenseSolver_JacobiSVD::Analyse(problem_t const& problem, utils::Opt const& options) {
  impl_.compute(problem.A_, Eigen::ComputeFullU | Eigen::ComputeFullV);
  AX_RETURN_OK();
}

LinsysSolveResult DenseSolver_BDCSVD::Solve(vecxr const& b, vecxr const&, utils::Opt const& options) {
  vecxr x = impl_.solve(b);
  return LinsysSolveResultImpl{std::move(x)};
}

Status DenseSolver_BDCSVD::Analyse(problem_t const& problem, utils::Opt const& options) {
  impl_.compute(problem.A_, Eigen::ComputeFullU | Eigen::ComputeFullV);
  AX_RETURN_OK();
}
}  // namespace ax::math