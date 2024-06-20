#include "ax/fem/timestepper/quasi_newton.hpp"

#include <tbb/parallel_for.h>

#include <exception>

#include "ax/fem/laplace_matrix.hpp"
#include "ax/math/linsys/preconditioner/IncompleteCholesky.hpp"
#include "ax/optim/common.hpp"
#include "ax/optim/optimizers/lbfgs.hpp"
#include "ax/utils/opt.hpp"

namespace ax::fem {

template <idx dim> Status fem::Timestepper_QuasiNewton<dim>::Initialize() {
  AX_RETURN_NOTOK(TimeStepperBase<dim>::Initialize());
  solver_ = math::SparseSolverBase::Create(math::SparseSolverKind::kConjugateGradient);
  // solver_->SetPreconditioner(std::make_unique<math::Preconditioner_IncompleteCholesky>());
  AX_RETURN_OK();
}
template <idx dim> void fem::Timestepper_QuasiNewton<dim>::UpdateSolverLaplace() {
  this->integration_scheme_->SetDeltaT(dt_back_);
  math::LinsysProblem_Sparse problem_sparse;
  const math::vec2r lame = this->u_lame_;
  real W = lame[0] + 2 * lame[1];
  // If you are using stable neohookean, you should bias the lambda and mu:
  real lambda = lame[0] + 5.0 / 6.0 * lame[1], mu = 4.0 / 3.0 * lame[1];
  W = 2 * mu + lambda;
  auto L = LaplaceMatrixCompute<dim>{*(this->mesh_)}(W);
  auto full_laplacian = this->integration_scheme_->ComposeHessian(this->mass_matrix_original_, L);
  problem_sparse.A_ = math::kronecker_identity<dim>(full_laplacian);
  problem_sparse.A_.makeCompressed();
  this->mesh_->FilterMatrixFull(problem_sparse.A_);
  // solver_->SetOptions({{"max_iter", 20}});
  solver_->Analyse(problem_sparse);
}

template <idx dim> void fem::Timestepper_QuasiNewton<dim>::BeginSimulation(real dt) {
  TimeStepperBase<dim>::BeginSimulation(dt);
  dt_back_ = dt;
  if (strategy_ == LbfgsStrategy::kLaplacian && dt_back_ > 0) {
    UpdateSolverLaplace();
  }
}

math::vecxr eigval;
math::matxxr eigvec;

template <idx dim> void fem::Timestepper_QuasiNewton<dim>::BeginTimestep(real dt) {
  TimeStepperBase<dim>::BeginTimestep(dt);
  if (strategy_ == LbfgsStrategy::kLaplacian) {
    if (dt_back_ != dt) {
      dt_back_ = dt;
      AX_LOG_FIRST_N(WARNING, 3)
          << "Update Laplace Solver due to timestep change."
             " (This is not recommended for LBFGS-PD because the laplace matrix will recompute.)";
      UpdateSolverLaplace();
    }
  } else if (strategy_ == LbfgsStrategy::kHard) {
    math::LinsysProblem_Sparse problem_sparse;
    problem_sparse.A_ = this->Hessian(this->du_inertia_);
    problem_sparse.A_.makeCompressed();

    this->mesh_->FilterMatrixFull(problem_sparse.A_);
    solver_->SetOptions({{"max_iter", 20}});
    solver_->Analyse(problem_sparse);
  } else if (strategy_ == LbfgsStrategy::kReservedForExperimental) {
    // Test this idea:
    auto A = this->Hessian(this->du_inertia_);
    auto eigsys = math::eig(A.toDense());
    eigvec = eigsys.first;
    eigval = eigsys.second;
  }
}

template <idx dim> void fem::Timestepper_QuasiNewton<dim>::SolveTimestep() {
  optim::Lbfgs &optimizer = optimizer_;
  optimizer.SetTolGrad(this->rel_tol_grad_);
  optimizer.SetTolVar(this->tol_var_);
  optimizer.SetMaxIter(this->max_iter_);
  if (strategy_ == LbfgsStrategy::kHard) {
    optimizer.SetApproxSolve(
        [&](math::vecxr const &g, math::vecxr const &, math::vecxr const &) -> math::vecxr {
          auto approx = solver_->Solve(g, g * this->dt_ * this->dt_);
          return std::move(approx.solution_);
        });
  } else if (strategy_ == LbfgsStrategy::kLaplacian) {
    optimizer.SetApproxSolve(
        [&](math::vecxr const &g, math::vecxr const &, math::vecxr const &) -> math::vecxr {
          auto approx = solver_->Solve(g, g * this->dt_ * this->dt_);
          return std::move(approx.solution_);
        });
  } else if (strategy_ == LbfgsStrategy::kReservedForExperimental) {
    optimizer_.SetApproxSolve(
        [&](math::vecxr const &g, math::vecxr const &, math::vecxr const &) -> math::vecxr {
          // decompose g to eigensystem:
          math::vecxr g_eig = eigvec.transpose() * g;
          math::vecxr to_divide = eigval;
          for (auto i = 0; i < to_divide.size(); ++i) {
            // add a random positive number to test the robustness of the approximation.
            auto &v = to_divide[i];
            real ratio = 0.5;
            // real ratio = std::lerp(0.1, 1, 1 - static_cast<real>(i) / to_divide.size());
            // real ratio = std::lerp(0.1, 1, static_cast<real>(i) / to_divide.size());
            v *= std::pow(10, std::lerp(-ratio, ratio, std::rand() / (real)RAND_MAX));
          }
          math::vecxr approx_eig = g_eig.array() / to_divide.array();
          math::vecxr ret = eigvec * approx_eig;
          return ret;
        });
  }

  auto problem = this->AssembleProblem();
  optim::OptResult result;
  try {
    result = optimizer.Optimize(problem, this->du_inertia_.reshaped());
  } catch (std::exception const &e) {
    AX_LOG(ERROR) << "Timestep solve failed: " << e.what();
    return;
  }
  if (result.converged_grad_) {
    AX_LOG(INFO) << "LBFGS iteration converged: gradient";
  } else if (result.converged_var_) {
    AX_LOG(INFO) << "LBFGS iteration converged: variance";
  } else {
    AX_LOG(ERROR) << "LBFGS iteration failed to converge!";
  }

  AX_LOG(WARNING) << "#Iter: " << result.n_iter_ << " iterations.";
  this->du_ = result.x_opt_.reshaped(dim, this->mesh_->GetNumVertices());
}

template <idx dim> void Timestepper_QuasiNewton<dim>::SetOptions(const utils::Opt &option) {
  auto [has_strategy, strategy] = utils::extract_enum<LbfgsStrategy>(option, "lbfgs_strategy");
  if (has_strategy) {
    if (strategy) {
      strategy_ = strategy.value();
    } else {
      AX_LOG(WARNING) << "Invalid lbfgs_strategy option: "
                      << option.at("lbfgs_strategy").as_string();
    }
  }

  auto [has_sparse_solver, sparse_solver]
      = utils::extract_enum<math::SparseSolverKind>(option, "sparse_solver");

  if (has_sparse_solver) {
    if (sparse_solver) {
      solver_ = math::SparseSolverBase::Create(sparse_solver.value());
    } else {
      AX_LOG(WARNING) << "Invalid sparse_solver option: " << option.at("sparse_solver").as_string();
    }
  }

  utils::extract_tunable(option, "sparse_solver_opt", solver_.get());
  utils::extract_tunable(option, "lbfgs_opt", &optimizer_);
  TimeStepperBase<dim>::SetOptions(option);
}

template <idx dim> utils::Opt Timestepper_QuasiNewton<dim>::GetOptions() const {
  auto option = TimeStepperBase<dim>::GetOptions();
  option["lbfgs_strategy"] = utils::reflect_name(strategy_).value();
  option["lbfgs_opt"] = optimizer_.GetOptions();
  if (solver_) {
    option["sparse_solver"] = utils::reflect_name(solver_->Kind()).value();
    option["sparse_solver_opt"] = solver_->GetOptions();
  }
  return option;
}

template class Timestepper_QuasiNewton<2>;
template class Timestepper_QuasiNewton<3>;

}  // namespace ax::fem
