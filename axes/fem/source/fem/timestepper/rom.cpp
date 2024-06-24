#include "ax/fem/timestepper/rom.hpp"

#include <tbb/parallel_for.h>

#include "ax/math/approx.hpp"
#include "ax/optim/optimizers/lbfgs.hpp"
#include "ax/optim/optimizers/newton.hpp"
#include "ax/optim/spsdm.hpp"
#include "ax/optim/spsdm/diagonal.hpp"
#include "ax/optim/spsdm/eigenvalue.hpp"
#include "ax/utils/time.hpp"
#undef ERROR

namespace ax::fem {

// template <idx dim> Status TimeStepper_ROM<dim>::Step(real dt) {
//   // Get the mesh
//   AX_TIME_FUNC();
//   auto &mesh = this->GetMesh();
//   auto &elasticity = this->GetElasticity();
//   elasticity.SetLame(this->u_lame_);
//   auto &velocity = this->velocity_;
//   auto &mass_matrix = this->mass_matrix_;
//   math::vec2r lame = this->u_lame_;
//   idx n_vert = mesh.GetNumVertices();
//   // Setup the NonLinear Problem.
//   optim::OptProblem problem;

//   // Setup the initial guess.
//   math::vecxr y = (latent_velocity_ * dt).reshaped();
//   math::fieldr<dim> full_space_inertia_position_expectation
//       = mesh.GetVertices() 
//         + dt * velocity
//         + dt * dt * this->ext_accel_ * this->mass_matrix_original_;

//   real max_tol = (mass_matrix * math::vecxr::Ones(n_vert * dim)).maxCoeff() + math::epsilon<real>;
//   AX_LOG(INFO) << "Max Tol: " << max_tol;
//   // Setup the objective function.
//   problem
//       .SetEnergy([&](const math::vecxr &w) -> real {
//         math::fieldr<dim> dl = w.reshaped(dim, latent_.cols());
//         math::fieldr<dim> x = LatentRestoreX(dl + latent_);
//         math::fieldr<dim> xy = (x - full_space_inertia_position_expectation);
//         real kinematic = 0.5 * xy.reshaped().dot(mass_matrix * xy.reshaped());
//         elasticity.Update(x, ElasticityUpdateLevel::kEnergy);
//         elasticity.UpdateEnergy();
//         real elastic = elasticity.GetEnergyOnElements().sum() * dt * dt;
//         return kinematic + elastic;
//       })
//       .SetGrad([&](const math::vecxr &w) -> math::vecxr {
//         math::fieldr<dim> dl = w.reshaped(dim, latent_.cols());
//         math::fieldr<dim> x = LatentRestoreX(dl + latent_);
//         math::fieldr<dim> xy = (x - full_space_inertia_position_expectation);
//         math::fieldr<dim> Mxy = xy * this->mass_matrix_original_;
//         elasticity.Update(x, ElasticityUpdateLevel::kStress);
//         elasticity.UpdateStress();
//         elasticity.GatherStressToVertices();
//         math::vecxr grad_full = (dt * dt * elasticity.GetStressOnVertices() + Mxy).reshaped();
//         mesh.FilterVector(grad_full, true);
//         math::vecxr grad_rom = (grad_full.reshaped(dim, n_vert) * basis_).reshaped();
//         return grad_rom;
//       })
//       .SetConvergeGrad([&](const math::vecxr &x, const math::vecxr & g) -> real {
//         real rv = math::abs(g).maxCoeff() / max_tol / (dt * dt);
//         return rv;
//       })
//       .SetVerbose([&](idx i, const math::vecxr &X, const real energy) {
//         AX_LOG(INFO) << "Iter: " << i << " Energy: " << energy
//                      << "|g|=" << problem.EvalGrad(X).norm() << "X=" << X.transpose();
//       });

//   optim::Lbfgs optimizer;
//   optimizer.SetTolGrad(0.04);
//   optimizer.SetMaxIter(1000);
//   auto result = optimizer.Optimize(problem, y);
//   if (!result.ok()) {
//     AX_LOG(WARNING) << "Optimizer: failed to compute! (not a convergency problem.)";
//     return result.status();
//   } else if (!(result->converged_grad_ || result->converged_var_)) {
//     AX_LOG(ERROR) << "Optimizer failed to converge!";
//   }
//   AX_LOG(WARNING) << "#Iter: " << result->n_iter_ << " iterations.";
//   auto x = LatentRestoreX(result->x_opt_.reshaped(dim, latent_.cols()) + latent_);
//   AX_CHECK_OK(mesh.SetVertices(x));
//   latent_velocity_ = result->x_opt_.reshaped(dim, latent_.cols()) / dt;
//   velocity = (x - LatentRestoreX(latent_)) / dt;
//   latent_ = result->x_opt_.reshaped(dim, latent_.cols()) + latent_;
//   AX_RETURN_OK();
// }

template<idx dim>
Status TimeStepper_ROM<dim>::Initialize() {
  x0_ = this->GetMesh()->GetVertices();
  return TimeStepperBase<dim>::Initialize();
}

template class TimeStepper_ROM<2>;
template class TimeStepper_ROM<3>;

}  // namespace ax::fem
