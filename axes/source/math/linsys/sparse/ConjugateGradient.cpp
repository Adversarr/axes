#include "axes/math/linsys/sparse/ConjugateGradient.hpp"

namespace ax::math {

Status SparseSolver_ConjugateGradient::Analyse(LinsysProblem_Sparse const &problem) {
  if (preconditioner_) {
    AX_RETURN_NOTOK(preconditioner_->Analyse(problem));
  } else {
    solver_.compute(problem.A_);
    if (solver_.info() != Eigen::Success) {
      return utils::FailedPreconditionError("SparseSolver_ConjugateGradient: factorization failed");
    }
  }

  AX_RETURN_OK();
}

LinsysSolveResult SparseSolver_ConjugateGradient::Solve(vecxr const &b, vecxr const &x0) {
  if (!preconditioner_) {
    // TODO: Fix bug.
    // solver_.setMaxIterations(options.GetDefault<idx>("max_iter", 100));
    vecxr x;
    if (x0.size() > 0) {
      if (x0.size() != b.size()) {
        return utils::FailedPreconditionError("Size mismatch!");
      }
      x = solver_.solveWithGuess(b, x0);
    } else {
      x = solver_.solve(b);
    }
    LinsysSolveResultImpl impl(x, solver_.info() == Eigen::Success);
    impl.num_iter_ = solver_.iterations();
    impl.l2_err_ = solver_.error();
    return impl;
  } else {
    AX_CHECK(false) << "This branch have not been implemented yet";
  }
}

Status SparseSolver_ConjugateGradient::SetOptions(utils::Opt const &opt) {
  AX_SYNC_OPT_IF(opt, idx, max_iter){
    if (max_iter_ < 1) {
      return utils::InvalidArgumentError("max_iter must be positive");
    }

    solver_.setMaxIterations(max_iter_);
  }

  AX_SYNC_OPT_IF(opt, real, tol){
    if (tol_ < 0) {
      return utils::InvalidArgumentError("tol must be non-negative");
    }
    solver_.setTolerance(tol_);
  }

  if (opt.Holds<std::string>("preconditioner")) {
    std::string precond = opt.Get<std::string>("preconditioner");
    auto kind = utils::reflect_enum<PreconditionerKind>(precond);
    if (!kind) {
      return utils::InvalidArgumentError("Invalid preconditioner kind: " + precond);
    }
    auto pc = PreconditionerBase::Create(kind.value());
    if (!pc) {
      return utils::InvalidArgumentError("Failed to create preconditioner: " + precond);
    }
    pc.swap(preconditioner_);
  }

  if (opt.Holds<utils::Opt>("preconditioner_options")) {
    return preconditioner_->SetOptions(opt.Get<utils::Opt>("preconditioner_options"));
  }

  AX_RETURN_OK();
}

utils::Opt SparseSolver_ConjugateGradient::GetOptions() const {
  utils::Opt opt {
    {"max_iter", max_iter_},
    {"tol", tol_},
  };
  if (preconditioner_) {
    auto name = utils::reflect_name(preconditioner_->Kind());
    if (!name) {
      AX_LOG(FATAL) << "Invalid preconditioner kind: " << static_cast<idx>(preconditioner_->Kind());
    }
    opt.Emplace("preconditioner", name.value());
    opt.Emplace("preconditioner_options", preconditioner_->GetOptions());
  }
  return opt;
}

}  // namespace ax::math
