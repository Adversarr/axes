#include "ax/fem/timestepper/quasi_newton.hpp"

#include <tbb/parallel_for.h>

#include "ax/fem/laplace_matrix.hpp"
#include "ax/math/linsys/preconditioner/Diagonal.hpp"
#include "ax/math/linsys/preconditioner/IncompleteCholesky.hpp"
#include "ax/optim/common.hpp"
#include "ax/optim/optimizers/lbfgs.hpp"
#include "ax/optim/spsdm/eigenvalue.hpp"
#include "ax/utils/iota.hpp"
#include "ax/utils/time.hpp"

namespace ax::fem {

template <idx dim> Status fem::Timestepper_QuasiNewton<dim>::Initialize() {
  AX_RETURN_NOTOK(TimeStepperBase<dim>::Initialize());
  solver_ = math::SparseSolverBase::Create(math::SparseSolverKind::kConjugateGradient);
  solver_->SetPreconditioner(std::make_unique<math::PreconditionerDiagonal>());
  AX_RETURN_OK();
}

// template <idx dim> Status fem::Timestepper_QuasiNewton<dim>::Step(real dt) {
//   // Get the mesh
//   AX_TIME_FUNC();
//   auto &mesh = this->GetMesh();
//   auto &elasticity = this->GetElasticity();
//   elasticity.SetLame(this->u_lame_);
//   auto &velocity = this->velocity_;
//   auto const &mass_matrix = this->mass_matrix_;
//   math::vec2r lame = this->u_lame_;

//   idx n_vert = mesh.GetNumVertices();

//   // Setup the NonLinear Problem.
//   optim::OptProblem problem;
//   math::fieldr<dim> const x_cur = mesh.GetVertices();
//   math::vecxr const v_flat = velocity.reshaped();
//   math::vecxr const a_flat = this->ext_accel_.reshaped();
//   math::vecxr y = dt * v_flat + dt * dt * a_flat;
//   mesh.FilterVector(y, true);
//   for (idx i = 0; i < n_vert; ++i) {
//     for (idx d = 0; d < dim; ++d) {
//       if (mesh.IsDirichletBoundary(i, d)) {
//         y(i * dim + d) = mesh.GetBoundaryValue(i, d) - x_cur(d, i);
//       }
//     }
//   }

//   math::vecxr eacc = this->ext_accel_.reshaped();
//   mesh.FilterVector(eacc, true);
//   real max_tol = (mass_matrix * math::vecxr::Ones(n_vert * dim)).maxCoeff() + math::epsilon<real>;
//   problem
//       .SetEnergy([&](math::vecxr const &dx) -> real {
//         AX_TIMEIT("Eval Energy");
//         math::fieldr<dim> x_new = dx.reshaped(dim, n_vert) + x_cur;
//         elasticity.Update(x_new, ElasticityUpdateLevel::kEnergy);
//         elasticity.UpdateEnergy();
//         real elasticity_energy = elasticity.GetEnergyOnElements().sum() * dt * dt;
//         math::vecxr x_y = dx - y;
//         real kinematic_energy = x_y.dot(mass_matrix * x_y) * 0.5;
//         return elasticity_energy + kinematic_energy;
//       })
//       .SetGrad([&](math::vecxr const &dx) -> math::vecxr {
//         AX_TIMEIT("Eval Grad");
//         math::fieldr<dim> x_new = dx.reshaped(dim, n_vert) + x_cur;
//         elasticity.Update(x_new, ElasticityUpdateLevel::kStress);
//         elasticity.UpdateStress();
//         elasticity.GatherStressToVertices();
//         math::fieldr<dim> neg_force = elasticity.GetStressOnVertices();
//         math::vecxr grad_kinematic = mass_matrix * (dx - y);
//         math::vecxr grad_elasticity = neg_force.reshaped() * (dt * dt);
//         math::vecxr grad = grad_elasticity + grad_kinematic;
//         mesh.FilterVector(grad, true);
//         return grad;
//       })
//       .SetSparseHessian([&](math::vecxr const &dx) -> math::sp_matxxr {
//         AX_TIMEIT("Eval Sparse Hessian");
//         math::fieldr<dim> x_new = dx.reshaped(dim, n_vert) + x_cur;
//         elasticity.Update(x_new, ElasticityUpdateLevel::kHessian);
//         elasticity.UpdateHessian(true);
//         elasticity.GatherHessianToVertices();
//         auto stiffness = elasticity.GetHessianOnVertices();
//         math::sp_matxxr hessian = mass_matrix + (dt * dt) * stiffness;
//         mesh.FilterMatrixFull(hessian);
//         return hessian;
//       })
//       .SetConvergeGrad([&](const math::vecxr &, const math::vecxr &grad) -> real {
//         real rv = math::abs(grad).maxCoeff() / max_tol / (dt * dt);
//         return rv;
//       })
//       .SetConvergeVar(nullptr);

//   optim::Lbfgs lbfgs;
//   lbfgs.SetTolGrad(0.02);
//   lbfgs.SetMaxIter(1000);

//   if (strategy_ == LbfgsStrategy::kHard) {
//     math::LinsysProblem_Sparse problem_sparse;
//     problem_sparse.A_ = problem.EvalSparseHessian(y);
//     AX_CHECK_OK(solver_->Analyse(problem_sparse));
//   }

//   if (strategy_ == LbfgsStrategy::kLaplacian) {
//     lbfgs.SetApproxSolve([&](math::vecxr const &g) -> math::vecxr {
//       auto approx = solver_->Solve(g, g * dt * dt);
//       AX_CHECK_OK(approx);
//       return std::move(approx->solution_);
//     });
//   } else if (strategy_ == LbfgsStrategy::kHard) {
//     lbfgs.SetApproxSolve([&](math::vecxr const &g) -> math::vecxr {
//       auto approx = solver_->Solve(g, g * dt * dt);
//       AX_CHECK_OK(approx);
//       return std::move(approx->solution_);
//     });
//   }

//   auto result = lbfgs.Optimize(problem, y);

//   if (!result.ok()) {
//     AX_LOG(WARNING) << "LBFGS iteration failed to compute! (not a convergency problem.)";
//     return result.status();
//   } else if (!(result->converged_grad_ || result->converged_var_)) {
//     AX_LOG(ERROR) << "LBFGS iteration failed to converge!";
//   }
//   AX_LOG(WARNING) << "#Iter: " << result->n_iter_ << " iterations.";
//   math::fieldr<dim> x_new = result->x_opt_.reshaped(dim, mesh.GetNumVertices()) + x_cur;
//   velocity = (x_new - x_cur) / dt;
//   AX_RETURN_NOTOK(mesh.SetVertices(mesh.GetVertices() + velocity * dt));
//   AX_RETURN_OK();
// }

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
  AX_CHECK_OK(solver_->SetOptions({{"max_iter", 20}}));
  AX_CHECK_OK(solver_->Analyse(problem_sparse));
}

template <idx dim> void fem::Timestepper_QuasiNewton<dim>::BeginSimulation(real dt) {
  TimeStepperBase<dim>::BeginSimulation(dt);
  dt_back_ = dt;
  if (strategy_ == LbfgsStrategy::kLaplacian && dt_back_ > 0) {
    UpdateSolverLaplace();
  }
}

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
    AX_CHECK_OK(solver_->SetOptions({{"max_iter", 20}}));
    AX_CHECK_OK(solver_->Analyse(problem_sparse));
  }
}

template <idx dim> void fem::Timestepper_QuasiNewton<dim>::SolveTimestep() {
  optim::Lbfgs optimizer;
  optimizer.SetTolGrad(this->rel_tol_grad_);
  optimizer.SetTolVar(this->tol_var_);
  optimizer.SetMaxIter(this->max_iter_);
  if (strategy_ == LbfgsStrategy::kHard) {
    optimizer.SetApproxSolve([&](math::vecxr const &g) -> math::vecxr {
      auto approx = solver_->Solve(g, g * this->dt_ * this->dt_);
      AX_CHECK_OK(approx);
      return std::move(approx->solution_);
    });
  } else if (strategy_ == LbfgsStrategy::kLaplacian) {
    optimizer.SetApproxSolve([&](math::vecxr const &g) -> math::vecxr {
      auto approx = solver_->Solve(g, g * this->dt_ * this->dt_);
      AX_CHECK_OK(approx);
      return std::move(approx->solution_);
    });
  }

  auto problem = this->AssembleProblem();
  auto result = optimizer.Optimize(problem, this->du_inertia_.reshaped());
  if (!result.ok()) {
    AX_LOG(WARNING) << "LBFGS iteration failed to compute! (not a convergency problem.)";
  } else if (!(result->converged_grad_ || result->converged_var_)) {
    AX_LOG(ERROR) << "LBFGS iteration failed to converge!";
  }

  AX_LOG(WARNING) << "#Iter: " << result->n_iter_ << " iterations.";
  this->du_ = result->x_opt_.reshaped(dim, this->mesh_->GetNumVertices());
}

template class Timestepper_QuasiNewton<2>;
template class Timestepper_QuasiNewton<3>;

}  // namespace ax::fem
