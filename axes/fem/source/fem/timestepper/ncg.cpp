#include "ax/fem/timestepper/ncg.hpp"

#include "ax/math/accessor.hpp"
#include "ax/math/io.hpp"
#include "ax/utils/ndrange.hpp"
#include "ax/math/utils/formatting.hpp"

namespace ax::fem {

template <int dim>
Timestepper_NonlinearCg<dim>::Timestepper_NonlinearCg(std::shared_ptr<TriMesh<dim>> mesh)
    : TimeStepperBase<dim>(std::move(mesh)),
      optimizer_(std::make_unique<optim::Optimizer_NonlinearCg>()) {
}



template <int dim>
Timestepper_NonlinearCg<dim>::~Timestepper_NonlinearCg() = default;

template <int dim>
void Timestepper_NonlinearCg<dim>::SolveTimestep() {
  auto problem = this->AssembleProblem();
  optimizer_->SetTolVar(this->tol_var_);
  optimizer_->SetTolGrad(this->rel_tol_grad_);
  optimizer_->SetMaxIter(this->max_iter_);

  // Solve the problem
  optim::Variable x0 = this->du_inertia_.reshaped();
  optim::OptResult result = optimizer_->Optimize(problem, x0);
  if (!result.converged_) {
    AX_CRITICAL("Nonlinear CG optimizer did not converge. {}", result.err_msg_);
    return;
  }

  optimizer_->SetPreconditioner(
      [this](const optim::Variable& x, const optim::Gradient& gradient) -> optim::Variable {
        return DoPreconditioning(x, gradient);
      });

  AX_INFO("NCG Converged in {:3} iterations, grad={}, var={}", result.n_iter_,
          result.converged_grad_, result.converged_var_);
  this->du_ = result.x_opt_.reshaped(dim, this->mesh_->GetNumVertices());
}

template <int dim>
void Timestepper_NonlinearCg<dim>::SetOptions(utils::Options const& opt) {
  TimeStepperBase<dim>::SetOptions(opt);
  if (const auto* it = opt.find("optimizer_opt"); it != opt.end()) {
    optimizer_->SetOptions(it->value().as_object());
  }
}

template <int dim>
void Timestepper_NonlinearCg<dim>::BeginSimulation(Real dt) {
  TimeStepperBase<dim>::BeginSimulation(dt);

  // Prepare my jacobi preconditioner
  jacob_.setZero(this->mesh_->GetNumVertices());
  // set mass diagonal:
  for (size_t i = 0; i < this->mesh_->GetNumVertices(); ++i) {
    jacob_[i] = this->mass_matrix_.coeff(i, i);
  }

  // Now it is N, 32 each column is a basis.
  basis_ = math::read_npy_v10_real("/home/adversarr/Repo/neural_subspace/basis.npy");
  // we assume the b.c. does not change in time
  for (auto [i, d]: utils::ndrange<Index>(this->mesh_->GetNumVertices(), dim)) {
    if (this->mesh_->IsDirichletBoundary(i, d)) {
      basis_.row(i).setZero(); // we now assume that, all the dof are fixed for this vertex
    }
  }

  // Make sure the basis is unit, ortho, we perform a Gram-Schmidt.
  for (size_t i = 0; i < basis_.cols(); ++i) {
    for (size_t j = 0; j < i; ++j) {
      basis_.col(i) -= basis_.col(j).dot(basis_.col(i)) * basis_.col(j);
    }
    basis_.col(i).normalize();
  }

  math::write_npy_v10("/home/adversarr/Repo/neural_subspace/basis_actual.npy", basis_);
}

template <int dim>
utils::Options Timestepper_NonlinearCg<dim>::GetOptions() const {
  auto opt = TimeStepperBase<dim>::GetOptions();
  opt["optimizer_opt"] = optimizer_->GetOptions();
  return opt;
}

template <int dim>
optim::Optimizer_NonlinearCg* Timestepper_NonlinearCg<dim>::GetOptimizer() {
  return optimizer_.get();
}

template <int dim>
optim::Variable Timestepper_NonlinearCg<dim>::DoPreconditioning(optim::Variable const& x0,
                                                                optim::Gradient const& gradient) {
  // Spread the gradient to each element, and then apply Additive Schwarz.
  // This is a simple preconditioner that can be improved.
  // x0: du.reshaped(dim, n_vert) + u_

  // the first version is approximated jacobi.
  // auto result = gradient;
  // for (size_t i = 0; i < this->mesh_->GetNumVertices(); ++i) {
  //   result.block<dim, 1>(i * dim, 0) /= jacob_[i];
  // }
  // return result;

  // Precondition by the basis:
  const auto& du = x0;
  Index num_vert = this->GetMesh()->GetNumVertices();
  auto hessian = this->Hessian(du.reshaped(dim, num_vert) + this->u_, true); // give me the SPSD hessian.
  math::RealMatrixX full_basis(num_vert * dim, 32); // (N*d, 32)
  for (Index d = 0; d < dim; ++d) {
    full_basis.block(d * num_vert, 0, num_vert, 32) = basis_ / 3;
  }

  // compute the reduced hessian, 32 by 32
  math::RealMatrixX reduced_hessian = full_basis.transpose() * hessian * full_basis;

  // we inspect its eigen system:
  Eigen::SelfAdjointEigenSolver<math::RealMatrixX> eigensolver(reduced_hessian);
  AX_INFO("Eigen Values are: {}", eigensolver.eigenvalues());

  // compute the reduced gradient, gradient is N d vector
  // then we multiply it with the basis, we get 32
  // r = b - Ax, Projection is P
  math::RealVectorX reduced_gradient = full_basis.transpose() * gradient; // P@r
  // Solve the reduced system
  math::RealVectorX reduced_result = reduced_hessian.ldlt().solve(reduced_gradient);

  // assemble the final result
  math::RealVectorX solved_part = full_basis * reduced_result; // P.T @ (P A^{-1} P.T) P @ r
  // unsolved part
  math::RealVectorX unsolved_part = gradient - full_basis * reduced_gradient; // (I - P.T @ P) @ r
  math::RealVectorX result = solved_part + unsolved_part;

  // check that result and gradient are in the same direction
  AX_INFO("Preconditioning: result={:12.6e}, gradient={:12.6e} dot(result, grad)={:12.6e}", result.norm(),
          gradient.norm(), math::dot(result, gradient));

  return result;
}

// Explicit instantiation
template class Timestepper_NonlinearCg<2>;
template class Timestepper_NonlinearCg<3>;

}  // namespace ax::fem