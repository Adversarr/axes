#include "ax/fem/timestepper/quasi_newton.hpp"

#include "ax/fem/elasticity/linear.hpp"
#include "ax/fem/elasticity/neohookean_bw.hpp"
#include "ax/optim/common.hpp"
#include "ax/optim/optimizers/lbfgs.hpp"
#include "ax/optim/spsdm/eigenvalue.hpp"

namespace ax::fem {

template<idx dim>
Status fem::Timestepper_QuasiNewton<dim>::Init(utils::Opt const &opt){
  AX_RETURN_NOTOK(TimeStepperBase<dim>::Init(opt));
  solver_ = math::SparseSolverBase::Create(math::SparseSolverKind::kLDLT);
  math::LinsysProblem_Sparse problem_sparse;
  problem_sparse.A_ = this->mass_matrix_;

  // Second part: K * dt * dt
  ElasticityCompute<dim, elasticity::Linear> elast(this->GetDeformation());
  elast.UpdateDeformationGradient(this->GetMesh().GetVertices(), DeformationGradientUpdate::kHessian);
  math::vec2r fake_lame = {this->lame_[0], this->lame_[1] * 2};
  idx n_dof = dim * this->GetMesh().GetNumVertices();
  problem_sparse.A_ += 1e-4 * math::make_sparse_matrix(n_dof, n_dof, 
                                            this->deform_->HessianToVertices(elast.Hessian(fake_lame)));

  this->mesh_->FilterMatrix(problem_sparse.A_);
  AX_RETURN_NOTOK(solver_->Analyse(problem_sparse));
  AX_RETURN_OK();
}

template <idx dim> Status fem::Timestepper_QuasiNewton<dim>::Step(real dt) {
  // Get the mesh
  auto &mesh = this->GetMesh();
  auto &deform = this->GetDeformation();
  auto &elasticity = this->GetElasticity();
  auto &velocity = this->velocity_;
  auto &mass_matrix = this->mass_matrix_;
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
        math::fieldr<dim> neg_force = deform.StressToVertices(stress_on_element);
        math::vecxr grad_kinematic = mass_matrix * (dx - y);
        math::vecxr grad_elasticity = neg_force.reshaped() * (dt * dt);
        math::vecxr grad = grad_elasticity + grad_kinematic;
        mesh.FilterVector(grad, true);
        return grad;
      })
      .SetConvergeGrad([&](const math::vecxr &, const math::vecxr &grad) -> real {
        real ext_force = eacc.dot(mass_matrix * eacc);
        real rel = grad.norm() / ext_force / (dt * dt);
        return rel;
      });
      // .SetVerbose([&](idx i, const math::vecxr& X, const real energy) {
      //   AX_LOG(INFO) << "Iter: " << i << " Energy: " << energy << "|g|=" << problem.EvalGrad(X).norm();
      // });

  optim::Lbfgs lbfgs;
  lbfgs.SetTolGrad(0.02);
  lbfgs.SetMaxIter(300);

  lbfgs.SetApproxSolve([&](math::vecxr const & g) -> math::vecxr {
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