#include "ax/fem/timestepper/quasi_newton.hpp"

#include <exception>

#include "ax/core/entt.hpp"
#include "ax/fem/laplace_matrix.hpp"
#include "ax/optim/common.hpp"
#include "ax/utils/opt.hpp"

namespace ax::fem {

template <int dim> void fem::Timestepper_QuasiNewton<dim>::Initialize() {
  TimeStepperBase<dim>::Initialize();
  if (!solver_) {
    AX_WARN("Use default sparse solver: Cholmod.");
    solver_ = math::SparseSolverBase::Create(math::SparseSolverKind::kCholmod);
  }
}

template <int dim> void fem::Timestepper_QuasiNewton<dim>::UpdateSolverLaplace() {
  const math::vec2r lame = this->u_lame_;
  // If you are using stable neohookean, you should bias the lambda and mu:
  // real lambda = lame[0] + 5.0 / 6.0 * lame[1], mu = 4.0 / 3.0 * lame[1];
  real lambda = lame[0], mu = lame[1];
  real W = 2 * mu + lambda;
  math::spmatr L = LaplaceMatrixCompute<dim>{*(this->mesh_)}(1.0);
  L *= W;
  auto full_laplacian = this->integration_scheme_->ComposeHessian(this->mass_matrix_original_, L);
  math::spmatr A = math::kronecker_identity<dim>(full_laplacian);
  this->mesh_->FilterMatrixFull(A);
  solver_->SetProblem(A).Compute();
}

template <int dim> math::spmatr Timestepper_QuasiNewton<dim>::GetLaplacianAsApproximation() const {
  const math::vec2r lame = this->u_lame_;
  real lambda = lame[0], mu = lame[1];
  real W = 2 * mu + lambda;
  auto L = LaplaceMatrixCompute<dim>{*(this->mesh_)}();
  L *= W;
  auto full_laplacian = this->integration_scheme_->ComposeHessian(this->mass_matrix_original_, L);
  this->mesh_->FilterMatrixDof(0, full_laplacian);
  return full_laplacian;
}

template <int dim> void fem::Timestepper_QuasiNewton<dim>::BeginSimulation(real dt) {
  TimeStepperBase<dim>::BeginSimulation(dt);
  UpdateSolverLaplace();
}

math::vecxr eigval;
math::matxxr eigvec;

template <int dim> void fem::Timestepper_QuasiNewton<dim>::BeginTimestep() {
  TimeStepperBase<dim>::BeginTimestep();
  if (strategy_ == LbfgsStrategy::kHard) {
    auto A = this->Hessian(this->du_inertia_);
    this->mesh_->FilterMatrixFull(A);
    solver_->SetProblem(std::move(A)).Compute();
  } else if (strategy_ == LbfgsStrategy::kReservedForExperimental) {
    AX_WARN("You are using the experimental mode!");
  }
}

template <int dim> void fem::Timestepper_QuasiNewton<dim>::SolveTimestep() {
  optim::Optimizer_Lbfgs &optimizer = optimizer_;
  optimizer.SetTolGrad(this->rel_tol_grad_);
  optimizer.SetTolVar(this->tol_var_);
  optimizer.SetMaxIter(this->max_iter_);

  if (strategy_ == LbfgsStrategy::kHard) {
    optimizer.SetApproxSolve(
        [&](math::vecxr const &g, math::vecxr const &, math::vecxr const &) -> math::vecxr {
          auto approx = solver_->Solve(g, g * this->dt_ * this->dt_);
          return approx.solution_;
        });
  } else if (strategy_ == LbfgsStrategy::kLaplacian) {
    optimizer.SetApproxSolve(
        [&](math::vecxr const &g, math::vecxr const &, math::vecxr const &) -> math::vecxr {
          auto approx = solver_->Solve(g, g * this->dt_ * this->dt_);
          return approx.solution_;
        });
  } else if (strategy_ == LbfgsStrategy::kReservedForExperimental) {
    optimizer.SetApproxSolve([&](math::vecxr const &g, math::vecxr const &s,
                                  math::vecxr const &y) -> math::vecxr {
      // Addtive Schwartz on each element.
    });
    // optimizer_.SetApproxSolve(
    //     [&](math::vecxr const &gk, math::vecxr const &sk, math::vecxr const &yk) -> math::vecxr {
    //       auto *cmpt = try_get_resource<SparseInverseApproximator>();
    //       AX_THROW_IF_NULL(cmpt, "SparseInverseApproximator not set.");
    //       return cmpt->A_ * gk;
    //       auto apply = [cmpt](math::vecxr const &v) -> math::vecxr {
    //         auto const &A = cmpt->A_;
    //         auto const &delta = cmpt->eig_modification_;
    //         math::vecxr At_v = A.transpose() * v;
    //         return A * At_v + delta * v;
    //       };
    //
    //       if (sk.size() == 0 || yk.size() == 0) {
    //         return apply(gk);
    //       }
    //
    //       if (!(cmpt->require_check_secant_)) {
    //         return apply(gk);
    //       }
    //
    //       // NOTE: Our original implementation in python
    //       // Hyk = self.apply_LDLT(y[-1])
    //       // gamma_Hsy = np.dot(y[-1], s[-1]) / (np.dot(y[-1], Hyk) + epsilon)
    //       // LDLT_q = self.apply_LDLT(q)
    //       // r = gamma_Hsy * LDLT_q
    //       // ---------------------------------------------------------------------
    //       // Derivation: Estimiate the secant equation coefficient
    //       // ---------------------------------------------------------------------
    //       // 1. Traditional. Estimate the 1 rank approximation of H0.
    //       // gamma_LSy = (np.dot(s[-1], y[-1]) / (np.dot(y[-1], y[-1]) + epsilon))
    //       // print(f'LSy: {gamma_LSy}')
    //       // 2. Ours. Estimate the approximation of H0, but with a different scale.
    //       // Secant equation: yk = Hk * sk
    //       //   <yk, sk> = gamma * <yk, I yk> => H0 = gamma I
    //       //   <yk, sk> = gamma * <yk, H yk> => gamma = <yk, sk> / <yk, H yk>
    //       real const gamma_Hsy = yk.dot(sk) / (yk.dot(apply(yk)) + math::epsilon<real>);
    //       return gamma_Hsy * apply(gk);
    //     });
  }

  auto problem = this->AssembleProblem();
  optim::OptResult result;
  try {
    result = optimizer.Optimize(problem, this->du_inertia_.reshaped());
  } catch (std::exception const &e) {
    AX_ERROR("Timestep solve failed: {}", e.what());
    return;
  }

  // SECT: Check the convergency result.
  if (!result.converged_) {
    AX_ERROR("Failed to converge, early stopped!");
  }

  AX_INFO("Converged: {}, Iterations={}", result.converged_var_ || result.converged_grad_,
          result.n_iter_);
  this->du_ = result.x_opt_.reshaped(dim, this->mesh_->GetNumVertices());
}

template <int dim> void Timestepper_QuasiNewton<dim>::SetOptions(const utils::Options &option) {
  utils::extract_enum(option, "lbfgs_strategy", strategy_);
  utils::extract_and_create<math::SparseSolverBase, math::SparseSolverKind>(option, "sparse_solver",
                                                                            solver_);
  utils::extract_tunable(option, "sparse_solver_opt", solver_.get());
  /* SECT: Lbfgs Options */
  utils::extract_tunable(option, "lbfgs_opt", &optimizer_);
  /* SECT: More Options */
  TimeStepperBase<dim>::SetOptions(option);
}

template <int dim> utils::Options Timestepper_QuasiNewton<dim>::GetOptions() const {
  auto option = TimeStepperBase<dim>::GetOptions();
  option["lbfgs_strategy"] = utils::reflect_name(strategy_).value();
  option["lbfgs_opt"] = optimizer_.GetOptions();
  if (solver_) {
    option["sparse_solver"] = utils::reflect_name(solver_->GetKind()).value();
    option["sparse_solver_opt"] = solver_->GetOptions();
  }
  return option;
}

template class Timestepper_QuasiNewton<2>;
template class Timestepper_QuasiNewton<3>;

}  // namespace ax::fem
