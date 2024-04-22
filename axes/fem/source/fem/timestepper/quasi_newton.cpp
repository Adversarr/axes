#include "ax/fem/timestepper/quasi_newton.hpp"

#include "ax/fem/laplace_matrix.hpp"
#include "ax/optim/common.hpp"
#include "ax/optim/optimizers/lbfgs.hpp"
#include "ax/optim/spsdm/eigenvalue.hpp"

#include <tbb/parallel_for.h>

namespace ax::fem {

template <idx dim> Status fem::Timestepper_QuasiNewton<dim>::Init(utils::Opt const &opt) {
  AX_RETURN_NOTOK(TimeStepperBase<dim>::Init(opt));
  solver_ = math::SparseSolverBase::Create(math::SparseSolverKind::kLDLT);
  math::LinsysProblem_Sparse problem_sparse;
  problem_sparse.A_ = this->mass_matrix_;

  // ElasticityCompute<dim, elasticity::Linear> elast(this->GetDeformation());
  // elast.UpdateDeformationGradient(this->GetMesh().GetVertices(),
  // DeformationGradientUpdate::kHessian); math::vec2r fake_lame = {this->lame_[0], this->lame_[1] *
  // 2}; idx n_dof = dim * this->GetMesh().GetNumVertices(); problem_sparse.A_ += 1e-4 *
  // math::make_sparse_matrix(n_dof, n_dof,
  //                                           this->deform_->HessianToVertices(elast.Hessian(fake_lame)));

  // They call you Laplace: M + dtSq * Lap.
  const math::vec2r lame = this->lame_;
  real W = lame[0] + 2 * lame[1];

  // If you are using stable neohookean, you should bias the lambda and mu:
  real lambda = lame[0] + 5.0/6.0 * lame[1], mu = 4.0/3.0 * lame[1];
  W = 2 * mu + lambda;

  auto L = LaplaceMatrixCompute<dim>{*(this->mesh_)}(W);
  problem_sparse.A_ = this->mass_matrix_ + 1e-4 * L;

  this->mesh_->FilterMatrix(problem_sparse.A_);
  AX_RETURN_NOTOK(solver_->Analyse(problem_sparse));
  AX_RETURN_OK();
}

template <idx dim> Status fem::Timestepper_QuasiNewton<dim>::Step(real dt) {
  // Get the mesh
  auto &mesh = this->GetMesh();
  auto &elasticity = this->GetElasticity();
  auto &velocity = this->velocity_;
  auto const &mass_matrix = this->mass_matrix_;
  math::vec2r lame = this->lame_;

  idx n_vert = mesh.GetNumVertices();

  // Setup the NonLinear Problem.
  optim::OptProblem problem;
  math::fieldr<dim> const x_cur = mesh.GetVertices();
  math::vecxr const v_flat = velocity.reshaped();
  math::vecxr const a_flat = this->ext_accel_.reshaped();
  math::vecxr y = dt * v_flat + dt * dt * a_flat;
  mesh.FilterVector(y, true);
  for (idx i = 0; i < n_vert; ++i) {
    for (idx d = 0; d < dim; ++d) {
      if (mesh.IsDirichletBoundary(i, d)) {
        y(i * dim + d) = mesh.GetBoundaryValue(i, d) - x_cur(d, i);
      }
    }
  }

  math::vecxr eacc = this->ext_accel_.reshaped();
  mesh.FilterVector(eacc, true);
  real max_tol = (mass_matrix * math::vecxr::Ones(n_vert * dim)).maxCoeff() + math::epsilon<real>;
  problem
      .SetEnergy([&](math::vecxr const &dx) -> real {
        math::fieldr<dim> x_new = dx.reshaped(dim, n_vert) + x_cur;
        elasticity.UpdateDeformationGradient(x_new, DeformationGradientUpdate::kEnergy);
        auto energy_on_element = elasticity.Energy(lame);
        real elasticity_energy = energy_on_element.sum() * (dt * dt);
        math::vecxr x_y = dx - y;
        real kinematic_energy = x_y.dot(mass_matrix * x_y) * 0.5;
        return elasticity_energy + kinematic_energy;
      })
      .SetGrad([&](math::vecxr const &dx) -> math::vecxr {
        math::fieldr<dim> x_new = dx.reshaped(dim, n_vert) + x_cur;
        elasticity.UpdateDeformationGradient(x_new, DeformationGradientUpdate::kStress);
        auto stress_on_element = elasticity.Stress(lame);
        math::fieldr<dim> neg_force = elasticity.GatherStress(stress_on_element);
        math::vecxr grad_kinematic = mass_matrix * (dx - y);
        math::vecxr grad_elasticity = neg_force.reshaped() * (dt * dt);
        math::vecxr grad = grad_elasticity + grad_kinematic;
        mesh.FilterVector(grad, true);
        return grad;
      })
      .SetSparseHessian([&](math::vecxr const &dx) -> math::sp_matxxr {
        math::fieldr<dim> x_new = dx.reshaped(dim, n_vert) + x_cur;
        elasticity.UpdateDeformationGradient(x_new, DeformationGradientUpdate::kHessian);
        auto hessian_on_element = elasticity.Hessian(lame);
        // Before move to vertex, make spsd.
        tbb::parallel_for(tbb::blocked_range<idx>(0, hessian_on_element.size(), 1000),
                          [&](const tbb::blocked_range<idx> &r) {
                            for (idx i = r.begin(); i < r.end(); ++i) {
                              optim::EigenvalueModification em;
                              em.min_eigval_ = dt * dt;
                              auto result = em.Modify(hessian_on_element[i]);
                              if (!result.ok()) {
                                AX_LOG(WARNING)
                                    << "Eigenvalue modification failed: " << result.status();
                              } else {
                                hessian_on_element[i] = result.value();
                              }
                            }
                          });
        auto stiffness = elasticity.GatherHessian(hessian_on_element);
        math::sp_matxxr hessian = mass_matrix + (dt * dt) * stiffness;
        mesh.FilterMatrix(hessian);
        return hessian;
      })
      .SetConvergeGrad([&](const math::vecxr &, const math::vecxr &grad) -> real {
        real rv = math::abs(grad).maxCoeff() / max_tol / (dt * dt);
        return rv;
      })
      .SetConvergeVar(nullptr);

  optim::Lbfgs lbfgs;
  lbfgs.SetTolGrad(0.02);
  lbfgs.SetMaxIter(300);

  // math::LinsysProblem_Sparse problem_sparse;
  // problem_sparse.A_ = problem.EvalSparseHessian(y);
  // AX_CHECK_OK(solver_->Analyse(problem_sparse));

  lbfgs.SetApproxSolve([&](math::vecxr const &g) -> math::vecxr {
    auto approx = solver_->Solve(g, g * dt * dt);
    AX_CHECK_OK(approx);
    math::vecxr result = approx->solution_;
    mesh.FilterVector(result, true);
    return result;
  });

  auto result = lbfgs.Optimize(problem, y);
  if (!result.ok()) {
    AX_LOG(WARNING) << "LBFGS iteration failed to compute! (not a convergency problem.)";
    return result.status();
  } else if (!(result->converged_grad_ || result->converged_var_)) {
    AX_LOG(ERROR) << "LBFGS iteration failed to converge!";
  }
  math::fieldr<dim> x_new = result->x_opt_.reshaped(dim, mesh.GetNumVertices()) + x_cur;
  velocity = (x_new - x_cur) / dt;
  AX_LOG(INFO) << "#Iter: " << result->n_iter_ << " iterations.";
  AX_RETURN_NOTOK(mesh.SetVertices(mesh.GetVertices() + velocity * dt));
  AX_RETURN_OK();
}

template class Timestepper_QuasiNewton<2>;
template class Timestepper_QuasiNewton<3>;

}  // namespace ax::fem
