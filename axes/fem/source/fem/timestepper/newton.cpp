#include "ax/fem/timestepper/newton.hpp"

#include "ax/math/approx.hpp"
#include "ax/optim/optimizers/lbfgs.hpp"
#include "ax/optim/optimizers/newton.hpp"
#include "ax/optim/spsdm.hpp"
#include "ax/optim/spsdm/diagonal.hpp"
namespace ax::fem {

template <idx dim> Status TimeStepperNewton<dim>::Step(real dt) {
  // Get the mesh
  auto &mesh = this->GetMesh();
  auto &deform = this->GetDeformation();
  auto &elasticity = this->GetElasticity();
  auto &velocity = this->velocity_;

  // Get the mass matrix
  auto &mass_matrix = this->mass_matrix_;
  math::vec2r lame = this->lame_;

  // Setup the NonLinear Problem.
  optim::OptProblem problem;
  math::fieldr<dim> const x_cur = mesh.GetVertices();
  math::vecxr const v_flat = velocity.reshaped();
  math::vecxr const a_flat = this->ext_accel_.reshaped();
  math::vecxr y = dt * v_flat + dt * dt * a_flat.reshaped();
  mesh.FilterVector(y, true);
  idx n_vert = mesh.GetNumVertices();

  problem
      .SetEnergy([&](math::vecxr const &dx) -> real {
        math::fieldr<dim> x_new = dx.reshaped(dim, n_vert) + x_cur;
        elasticity.UpdateDeformationGradient(x_new, DeformationGradientUpdate::kEnergy);
        auto energy_on_element = elasticity.Energy(lame);
        real elasticity_energy = energy_on_element.sum();
        math::vecxr x_y = dx - y;
        real kinematic_energy = x_y.dot(mass_matrix * x_y) * 0.5 / dt / dt;
        return elasticity_energy + kinematic_energy;
      })
      .SetGrad([&](math::vecxr const &dx) -> math::vecxr {
        math::fieldr<dim> x_new = dx.reshaped(dim, n_vert) + x_cur;
        elasticity.UpdateDeformationGradient(x_new, DeformationGradientUpdate::kStress);
        auto stress_on_element = elasticity.Stress(lame);
        math::fieldr<dim> neg_force = deform.StressToVertices(stress_on_element);
        math::vecxr grad_kinematic = mass_matrix * (dx - y);
        math::vecxr grad_elasticity = dt * dt * neg_force.reshaped();
        mesh.FilterVector(grad_kinematic, true);
        mesh.FilterVector(grad_elasticity, true);
        math::vecxr grad = grad_elasticity + grad_kinematic;
        return grad;
      })
      .SetSparseHessian([&](math::vecxr const &dx) -> math::sp_matxxr {
        math::fieldr<dim> x_new = dx.reshaped(dim, n_vert) + x_cur;
        elasticity.UpdateDeformationGradient(x_new, DeformationGradientUpdate::kHessian);
        auto hessian_on_element = elasticity.Hessian(lame);
        auto hessian_on_vertice = deform.HessianToVertices(hessian_on_element);
        math::sp_matxxr stiffness = math::make_sparse_matrix(dim * n_vert, dim * n_vert, 
                                                             hessian_on_vertice);
        math::sp_matxxr hessian = mass_matrix + (dt * dt) * stiffness;
        mesh.FilterMatrix(hessian);
        return hessian;
      }).SetVerbose([&](idx i, const math::vecxr& dx, const real v){
        AX_LOG(INFO) << "Newton iteration " << i << " energy: " << v << " |grad|: " << math::norm(problem.EvalGrad(dx));
      }).SetConvergeGrad([&](const math::vecxr& dx, const math::vecxr& grad) -> real {
        math::vecxr eacc = this->ext_accel_.reshaped();
        mesh.FilterVector(eacc, true);
        real ext_force = eacc.dot(mass_matrix * eacc);
        real rel = grad.norm() / ext_force / (dt * dt);
        std::cout << "RelError = " << rel << std::endl;
        return rel;
      }).SetConvergeVar(nullptr);

  optim::Newton newton;
  newton.SetTolGrad(0.002);
  // auto result = newton.Optimize(problem, y);
  auto result = newton.Optimize(problem, math::vecxr::Zero(y.size()));
  if (!result.ok()) {
    AX_LOG(WARNING) << "Newton iteration failed to compute! (not a convergency problem.)";
    return result.status();
  } else if (!(result->converged_grad_ || result->converged_var_)) {
    AX_LOG(ERROR) << "Newton iteration failed to converge!";
  }
  math::fieldr<dim> x_new = result->x_opt_.reshaped(dim, mesh.GetNumVertices()) + x_cur;
  velocity = (x_new - x_cur) / dt;
  // AX_LOG(INFO) << "Newton Optimizer Options: " << newton.GetOptions();
  AX_LOG(INFO) << "#Iter: " << result->n_iter_ << " iterations.";
  AX_LOG(INFO) << "Optimal value: " << result->f_opt_;
  AX_LOG(INFO) << "Optimal |v|: " << math::norm(result->x_opt_);
  AX_LOG(INFO) << "Optimal |g|: " << problem.EvalGrad(result->x_opt_).norm();
  // AX_LOG(INFO) << "Optimal g: " << problem.EvalGrad(result->x_opt_).transpose();
  AX_RETURN_NOTOK(mesh.SetVertices(mesh.GetVertices() + velocity * dt));
  AX_RETURN_OK();
}

template class TimeStepperNewton<2>;
template class TimeStepperNewton<3>;

}  // namespace ax::fem