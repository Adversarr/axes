#include "ax/fem/timestepper/rom.hpp"

#include <tbb/parallel_for.h>

#include "ax/math/utils/approx.hpp"
#include "ax/optim/optimizers/lbfgs.hpp"
#include "ax/optim/optimizers/newton.hpp"
#include "ax/optim/spsdm.hpp"
#include "ax/optim/spsdm/diagonal.hpp"
#include "ax/optim/spsdm/eigenvalue.hpp"
#include "ax/utils/time.hpp"
#undef ERROR

namespace ax::fem {

// template <int dim> Status TimeStepper_ROM<dim>::Step(Real dt) {
//   // Get the mesh
//   AX_TIME_FUNC();
//   auto &mesh = this->GetMesh();
//   auto &elasticity = this->GetElasticity();
//   elasticity.SetLame(this->u_lame_);
//   auto &velocity = this->velocity_;
//   auto &mass_matrix = this->mass_matrix_;
//   math::RealVector2 lame = this->u_lame_;
//   Index n_vert = mesh.GetNumVertices();
//   // Setup the NonLinear Problem.
//   optim::OptProblem problem;

//   // Setup the initial guess.
//   math::RealVectorX y = (latent_velocity_ * dt).reshaped();
//   math::RealField<dim> full_space_inertia_position_expectation
//       = mesh.GetVertices() 
//         + dt * velocity
//         + dt * dt * this->ext_accel_ * this->mass_matrix_original_;

//   Real max_tol = (mass_matrix * math::RealVectorX::Ones(n_vert * dim)).maxCoeff() + math::epsilon<Real>;
//   AX_LOG(INFO) << "Max Tol: " << max_tol;
//   // Setup the objective function.
//   problem
//       .SetEnergy([&](const math::RealVectorX &w) -> Real {
//         math::RealField<dim> dl = w.reshaped(dim, latent_.cols());
//         math::RealField<dim> x = LatentRestoreX(dl + latent_);
//         math::RealField<dim> xy = (x - full_space_inertia_position_expectation);
//         Real kinematic = 0.5 * xy.reshaped().dot(mass_matrix * xy.reshaped());
//         elasticity.Update(x, ElasticityUpdateLevel::kEnergy);
//         elasticity.UpdateEnergy();
//         Real elastic = elasticity.GetEnergyOnElements().sum() * dt * dt;
//         return kinematic + elastic;
//       })
//       .SetGrad([&](const math::RealVectorX &w) -> math::RealVectorX {
//         math::RealField<dim> dl = w.reshaped(dim, latent_.cols());
//         math::RealField<dim> x = LatentRestoreX(dl + latent_);
//         math::RealField<dim> xy = (x - full_space_inertia_position_expectation);
//         math::RealField<dim> Mxy = xy * this->mass_matrix_original_;
//         elasticity.Update(x, ElasticityUpdateLevel::kStress);
//         elasticity.UpdateStress();
//         elasticity.GatherStressToVertices();
//         math::RealVectorX grad_full = (dt * dt * elasticity.GetStressOnVertices() + Mxy).reshaped();
//         mesh.FilterVector(grad_full, true);
//         math::RealVectorX grad_rom = (grad_full.reshaped(dim, n_vert) * basis_).reshaped();
//         return grad_rom;
//       })
//       .SetConvergeGrad([&](const math::RealVectorX &x, const math::RealVectorX & g) -> Real {
//         Real rv = math::abs(g).maxCoeff() / max_tol / (dt * dt);
//         return rv;
//       })
//       .SetVerbose([&](Index i, const math::RealVectorX &X, const Real energy) {
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

template<int dim>
void TimeStepper_ROM<dim>::Initialize() {
  x0_ = this->GetMesh()->GetVertices();
  return TimeStepperBase<dim>::Initialize();
}

template class TimeStepper_ROM<2>;
template class TimeStepper_ROM<3>;

}  // namespace ax::fem
