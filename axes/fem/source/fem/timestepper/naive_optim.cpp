#include "ax/fem/timestepper/naive_optim.hpp"
#include "ax/optim/spsdm/eigenvalue.hpp"

#include "ax/math/approx.hpp"
#include "ax/optim/optimizers/lbfgs.hpp"
#include "ax/optim/optimizers/newton.hpp"
#include "ax/optim/spsdm.hpp"
#include "ax/optim/spsdm/diagonal.hpp"
#include <tbb/parallel_for.h>
#undef ERROR

namespace ax::fem {

template <idx dim> Status Timestepper_NaiveOptim<dim>::Step(real dt) {
  // Get the mesh
  auto &mesh = this->GetMesh();
  auto &elasticity = this->GetElasticity();
  elasticity.SetLame(this->lame_);
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
  real max_tol = (mass_matrix * math::vecxr::Ones(n_vert * dim)).maxCoeff() + math::epsilon<real>;
  problem
      .SetEnergy([&](math::vecxr const &dx) -> real {
        math::fieldr<dim> x_new = dx.reshaped(dim, n_vert) + x_cur;
        elasticity.Update(x_new, ElasticityUpdateLevel::kEnergy);
        elasticity.UpdateEnergy();
        auto energy_on_element = elasticity.GetEnergyOnElements();
        real elasticity_energy = energy_on_element.sum() * dt * dt;
        math::vecxr x_y = dx - y;
        real kinematic_energy = x_y.dot(mass_matrix * x_y) * 0.5;
        return elasticity_energy + kinematic_energy;
      })
      .SetGrad([&](math::vecxr const &dx) -> math::vecxr {
        math::fieldr<dim> x_new = dx.reshaped(dim, n_vert) + x_cur;
        elasticity.Update(x_new, ElasticityUpdateLevel::kStress);
        elasticity.UpdateStress();
        elasticity.GatherStressToVertices();
        math::fieldr<dim> neg_force = elasticity.GetStressOnVertices();
        math::vecxr grad_kinematic = mass_matrix * (dx - y);
        math::vecxr grad_elasticity = dt * dt * neg_force.reshaped();
        math::vecxr grad = grad_elasticity + grad_kinematic;
        mesh.FilterVector(grad, true);
        return grad;
      })
      .SetSparseHessian([&](math::vecxr const &dx) -> math::sp_matxxr {
        math::fieldr<dim> x_new = dx.reshaped(dim, n_vert) + x_cur;
        elasticity.Update(x_new, ElasticityUpdateLevel::kHessian);
        elasticity.UpdateHessian(true);
        elasticity.GatherHessianToVertices();
        auto stiffness = elasticity.GetHessianOnVertices();
        math::sp_matxxr hessian = mass_matrix + (dt * dt) * stiffness;
        mesh.FilterMatrixFull(hessian);
        return hessian;
      })
      .SetConvergeGrad([&](const math::vecxr&, const math::vecxr& grad) -> real {
        real rv = math::abs(grad).maxCoeff() / max_tol / (dt * dt);
        return rv;
      })
      .SetConvergeVar(nullptr);
      // .SetVerbose([&](idx i, const math::vecxr& X, const real energy) {
      //   AX_LOG(INFO) << "Iter: " << i << " Energy: " << energy << "|g|=" << problem.EvalGrad(X).norm();
      // });

  optim::Newton optimizer;
  optimizer.SetTolGrad(0.02);
  optimizer.SetMaxIter(1000);
  auto result = optimizer.Optimize(problem, y);
  if (!result.ok()) {
    AX_LOG(WARNING) << "Optimizer: failed to compute! (not a convergency problem.)";
    return result.status();
  } else if (!(result->converged_grad_ || result->converged_var_)) {
    AX_LOG(ERROR) << "Optimizer failed to converge!";
  }
  math::fieldr<dim> x_new = result->x_opt_.reshaped(dim, mesh.GetNumVertices()) + x_cur;
  velocity = (x_new - x_cur) / dt;
  AX_LOG(WARNING) << "#Iter: " << result->n_iter_ << " iterations.";
  AX_RETURN_NOTOK(mesh.SetVertices(mesh.GetVertices() + velocity * dt));
  AX_RETURN_OK();
}

template class Timestepper_NaiveOptim<2>;
template class Timestepper_NaiveOptim<3>;

}  // namespace ax::fem
