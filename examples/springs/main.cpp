#include <imgui.h>
#include <implot.h>

#include <ax/gl/colormap.hpp>
#include <ax/gl/events.hpp>
#include <ax/gl/extprim/grid.hpp>
#include <ax/gl/primitives/lines.hpp>
#include <ax/utils/ndrange.hpp>
#include <range/v3/view/enumerate.hpp>

#include "ax/core/entt.hpp"
#include "ax/core/init.hpp"
#include "ax/core/logging.hpp"
#include "ax/gl/utils.hpp"
#include "ax/math/utils/formatting.hpp"
#include "ax/math/utils/structure_binding.hpp"
#include "ax/math/views.hpp"
#include "ax/optim/common.hpp"
#include "ax/optim/optimizers/lbfgs.hpp"
#include "ax/optim/optimizers/newton.hpp"
#include "ax/optim/optimizers/pcg.hpp"
#include "ax/utils/time.hpp"
using namespace ax;

// Minimize the Incremental Potential:
// E = 1/2dt^2 ||x - y||_M^2 + ||f||_M^2
bool better_initial_guess = false;
bool use_4th_model = false;
bool incremental = false;
int max_iteration = 10;

std::vector<Real> gradient_norm;

struct MassSpring {
  math::RealField3 position_;
  math::IndexField2 springs_;
  math::RealField1 original_length_;
  math::RealField1 stiffness_;
  math::RealField1 mass_;

  math::RealField3 last_position_;
  math::RealField3 next_position_;
  math::RealField3 external_force_;
  Real dt_{1e-2};

  std::vector<Index> fixed_vertices_;
  std::vector<bool> is_fixed_;
};

Real energy_quad(MassSpring const& sys, math::RealField3 const& x) {
  Real total = 0;
  math::RealField3 y = 2 * sys.position_ - sys.last_position_;
  auto y_accessor = math::make_accessor(y);
  auto x_accessor = math::make_accessor(x);
  for (auto [i, yi] : math::enumerate(y_accessor)) {
    yi += sys.dt_ * sys.dt_ * sys.external_force_.col(i) / sys.mass_(i);
    auto xi = x_accessor(i);
    auto diff = yi - xi;

    total += 0.5 * sys.mass_(i) * diff.squaredNorm();
  }
  // Quadratic model: E(x1, x2) = 1/2 k (|x1-x2| - l)^2
  auto springs_accessor = math::make_accessor(sys.springs_);
  for (auto [i, spring] : math::enumerate(springs_accessor)) {
    auto [a, b] = math::unpack(spring.eval());
    math::RealVector3 diff = x_accessor(a) - x_accessor(b);
    Real l = diff.norm() - sys.original_length_(i);
    Real stiffness = sys.stiffness_(i);
    total += 0.5 * stiffness * l * l * sys.dt_ * sys.dt_;
  }

  return total;
}

math::RealField3 gradient_quad(MassSpring const& sys, math::RealField3 const& x) {
  math::RealField3 grad = math::RealField3::Zero(x.rows(), x.cols());
  math::RealField3 y = 2 * sys.position_ - sys.last_position_;
  auto y_accessor = math::make_accessor(y);
  auto x_accessor = math::make_accessor(x);
  auto grad_accessor = math::make_accessor(grad);
  for (auto [i, yi] : math::enumerate(y_accessor)) {
    yi += sys.dt_ * sys.dt_ * sys.external_force_.col(i) / sys.mass_(i);
    auto xi = x_accessor(i);
    auto diff = yi - xi;
    grad_accessor(i) += sys.mass_(i) * diff;
  }

  // pE/px1 = k (|x1-x2| - l) * (x1 - x2) / |x1 - x2|
  auto springs_accessor = math::make_accessor(sys.springs_);
  for (auto [i, spring] : math::enumerate(springs_accessor)) {
    auto [a, b] = math::unpack(spring.eval());
    auto diff = x_accessor(a) - x_accessor(b);
    auto length = diff.norm();
    auto original_length = sys.original_length_(i);
    auto l = length - original_length;
    auto stiffness = sys.stiffness_(i);
    math::RealVector3 grad_a = stiffness * l * diff / length;
    grad_accessor(a) += grad_a * sys.dt_ * sys.dt_;
    grad_accessor(b) -= grad_a * sys.dt_ * sys.dt_;
  }

  for (auto i : sys.fixed_vertices_) {
    grad_accessor(i).setZero();
  }
  return grad;
}

Real grad_inf_norm(MassSpring const& sys, math::RealField3 const& x) {
  math::RealField3 grad = gradient_quad(sys, x);
  Real inf_norm = grad.cwiseAbs().maxCoeff();
  return inf_norm;
}

Real grad_l2_norm(MassSpring const& sys, math::RealField3 const& x) {
  math::RealField3 grad = gradient_quad(sys, x);
  Real l2_norm = grad.norm();
  return l2_norm;
}

Real energy_4th(MassSpring const& sys, math::RealField3 const& x) {
  Real total = 0;
  // Inertia term: y = 2 * last - current + dt^2 * f / m
  math::RealField3 y = 2 * sys.position_ - sys.last_position_;
  auto y_accessor = math::make_accessor(y);
  auto x_accessor = math::make_accessor(x);
  for (auto [i, yi] : math::enumerate(y_accessor)) {
    yi += sys.dt_ * sys.dt_ * sys.external_force_.col(i) / sys.mass_(i);
    auto xi = x_accessor(i);
    auto diff = yi - xi;

    total += 0.5 * sys.mass_(i) * diff.squaredNorm();
  }
  total /= sys.dt_ * sys.dt_;
  // Potential energy: V = 0.5 * k * (||y||^2 - l^2)^2
  auto springs_accessor = math::make_accessor(sys.springs_);
  for (auto [i, spring] : math::enumerate(springs_accessor)) {
    auto [a, b] = math::unpack(spring.eval());
    math::RealVector3 diff = x_accessor(a) - x_accessor(b);
    auto length = diff.squaredNorm();
    auto original_length = sys.original_length_(i);
    auto l2 = length - original_length * original_length;
    auto stiffness = sys.stiffness_(i);
    total += 0.25 * stiffness * l2 * l2;
  }
  return total;
}

math::RealField3 gradient_4th(MassSpring const& sys, math::RealField3 const& x) {
  math::RealField3 grad = math::RealField3::Zero(x.rows(), x.cols());
  // Inertia term: y = 2 * last - current + dt^2 * f / m
  math::RealField3 y = 2 * sys.position_ - sys.last_position_;
  auto y_accessor = math::make_accessor(y);
  auto x_accessor = math::make_accessor(x);
  auto grad_accessor = math::make_accessor(grad);
  for (auto [i, yi] : math::enumerate(y_accessor)) {
    yi += sys.dt_ * sys.dt_ * sys.external_force_.col(i) / sys.mass_(i);
    auto xi = x_accessor(i);
    auto diff = xi - yi;
    grad_accessor(i) += sys.mass_(i) * diff;
  }
  grad /= sys.dt_ * sys.dt_;
  // Potential energy: V = 0.5 * k * (||y|| - l)^2
  auto springs_accessor = math::make_accessor(sys.springs_);
  for (auto [i, spring] : math::enumerate(springs_accessor)) {
    auto [a, b] = math::unpack(spring.eval());
    auto diff = x_accessor(a) - x_accessor(b);
    auto length = diff.squaredNorm();
    auto original_length = sys.original_length_(i);

    auto l2 = length - original_length * original_length;
    auto stiffness = sys.stiffness_(i);
    grad_accessor(a) += stiffness * l2 * diff;
    grad_accessor(b) -= stiffness * l2 * diff;
  }

  for (auto i : sys.fixed_vertices_) {
    grad_accessor(i).setZero();
  }
  return grad;
}

math::RealMatrix<6, 6> local_hessian_4th(math::RealVector3 const& xa, math::RealVector3 const& xb,
                                         Real rho, Real k, Real original_length) {
  math::RealMatrix<6, 6> hessian;
  math::RealVector3 diff = xa - xb;
  Real length = diff.squaredNorm();
  Real l2 = length - original_length * original_length;

  //      E = 1/4 k  (|a-b|^2 - l)^2
  // diff a =     k  (|a-b|^2 - l) * (a - b)
  // diff b =     k  (|a-b|^2 - l) * (b - a)
  // H  a,a =     k ((|a-b|^2 - l) I + 2 * (a - b) * (a - b)^T)
  // H  b,b =     k ((|a-b|^2 - l) I + 2 * (b - a) * (b - a)^T)
  // H  a,b =     k (-(|a-b|^2 -l) I - 2 * (a - b) * (a - b)^T)
  math::RealMatrix<3, 3> I = math::RealMatrix<3, 3>::Identity();  // I
  math::RealMatrix<3, 3> outer = diff * diff.transpose();         // (a-b) (a-b)^T

  hessian.block<3, 3>(0, 0) = k * l2 * I + 2 * k * outer;
  hessian.block<3, 3>(3, 3) = k * l2 * I + 2 * k * outer;
  hessian.block<3, 3>(0, 3) = -k * l2 * I - 2 * k * outer;
  hessian.block<3, 3>(3, 0) = -k * l2 * I - 2 * k * outer;

  Eigen::SelfAdjointEigenSolver<math::RealMatrix<6, 6>> solver(hessian);
  math::RealVector<6> eigvals = solver.eigenvalues();
  for (auto& val : eigvals) {
    if (val < 0) {
      val = 0;
    }
  }
  hessian = solver.eigenvectors() * eigvals.asDiagonal() * solver.eigenvectors().transpose();

  // add a small value to the diagonal to avoid singularity
  hessian += hessian.Identity() * rho;
  return hessian;
}

math::RealSparseMatrix global_hessian_4th(MassSpring const& sys, math::RealField3 const& x) {
  // Potential term
  math::RealSparseCOO coo;
  auto springs_accessor = math::make_accessor(sys.springs_);
  for (auto [i, ij] : math::enumerate(springs_accessor)) {
    auto [a, b] = math::unpack(ij.eval());
    auto xa = x.col(a), xb = x.col(b);
    auto original_length = sys.original_length_(i);
    auto stiffness = sys.stiffness_(i);

    auto local = local_hessian_4th(xa, xb, 0, stiffness, original_length);
    for (auto [vi, vj] : utils::ndrange<Index>(2, 2)) {
      for (auto [di, dj] : utils::ndrange<Index>(3, 3)) {
        Index row = 3 * ij(vi) + di;
        Index col = 3 * ij(vj) + dj;
        Real coef = local(3 * vi + di, 3 * vj + dj);

        coo.emplace_back(row, col, coef);
      }
    }
  }

  // Inertia term, mass matrix.
  for (Index i = 0; i < x.cols(); ++i) {
    for (Index d = 0; d < 3; ++d) {
      coo.emplace_back(i * 3 + d, i * 3 + d, sys.mass_(0, i) / sys.dt_ / sys.dt_);
    }
  }

  math::RealSparseMatrix hessian = math::make_sparse_matrix(x.size(), x.size(), coo);
  math::for_each_entry(hessian, [&sys](Index i, Index j, Real& val) {
    Index vi = i / 3, vj = j / 3;
    if (std::find(sys.fixed_vertices_.begin(), sys.fixed_vertices_.end(), vi)
        != sys.fixed_vertices_.end()) {
      val = i == j ? 1. : 0.;
    } else if (std::find(sys.fixed_vertices_.begin(), sys.fixed_vertices_.end(), vj)
               != sys.fixed_vertices_.end()) {
      val = i == j ? 1. : 0.;
    }
  });
  hessian.prune(0.);
  hessian.makeCompressed();
  return hessian;
}

math::RealMatrix<6, 6> local_hessian_quad(math::RealVector3 const& xa, math::RealVector3 const& xb,
                                          Real rho, Real k, Real original_length) {
  math::RealMatrix<6, 6> hessian;
  math::RealMatrix<3, 3> I = math::RealMatrix<3, 3>::Identity();  // I
  math::RealVector3 diff = xa - xb;
  math::RealVector3 unit = diff.normalized();
  Real length = diff.norm();
  math::RealMatrix<3, 3> blocks;
  // blocks = k * (original_length / length - 1) * I -
  //          k * original_length * unit * unit.transpose() / length;
  blocks = unit * unit.transpose() + (length - original_length) * (I - unit * unit.transpose()) / length;
  blocks *= k;
  hessian.block<3, 3>(0, 0) = blocks;
  hessian.block<3, 3>(3, 3) = blocks;
  hessian.block<3, 3>(0, 3) = -blocks;
  hessian.block<3, 3>(3, 0) = -blocks;

  Eigen::SelfAdjointEigenSolver<math::RealMatrix<6, 6>> solver(hessian);
  math::RealVector<6> eigvals = solver.eigenvalues();
  for (auto& val : eigvals) {
    if (val < 0) {
      val = 0;
    }
  }
  hessian = solver.eigenvectors() * eigvals.asDiagonal() * solver.eigenvectors().transpose();

  return hessian;
}


math::RealSparseMatrix global_hessian_quad(MassSpring const& sys, math::RealField3 const& x) {
  math::RealSparseCOO coo;
  // Potential term
  auto springs_accessor = math::make_accessor(sys.springs_);
  for (auto [i, ij] : math::enumerate(springs_accessor)) {
    auto [a, b] = math::unpack(ij.eval());
    auto xa = x.col(a), xb = x.col(b);
    auto original_length = sys.original_length_(i);
    auto stiffness = sys.stiffness_(i) * sys.dt_ * sys.dt_;
    math::RealMatrix<6, 6> local_hessian = local_hessian_quad(xa, xb, 0, stiffness, original_length);

    for (auto [vi, vj] : utils::ndrange<Index>(2, 2)) {
      for (auto [di, dj] : utils::ndrange<Index>(3, 3)) {
        Index row = 3 * ij(vi) + di;
        Index col = 3 * ij(vj) + dj;
        Real coef = local_hessian(3 * vi + di, 3 * vj + dj);

        coo.emplace_back(row, col, coef);
      }
    }
  }

  // Inertia term, mass matrix.
  for (Index i = 0; i < x.cols(); ++i) {
    for (Index d = 0; d < 3; ++d) {
      coo.emplace_back(i * 3 + d, i * 3 + d, sys.mass_(0, i));
    }
  }

  math::RealSparseMatrix hessian = math::make_sparse_matrix(x.size(), x.size(), coo);
  math::for_each_entry(hessian, [&sys](Index i, Index j, Real& val) {
    Index vi = i / 3, vj = j / 3;
    if (std::find(sys.fixed_vertices_.begin(), sys.fixed_vertices_.end(), vi)
        != sys.fixed_vertices_.end()) {
      val = i == j ? 1. : 0.;
    } else if (std::find(sys.fixed_vertices_.begin(), sys.fixed_vertices_.end(), vj)
               != sys.fixed_vertices_.end()) {
      val = i == j ? 1. : 0.;
    }
  });
  hessian.prune(0.);
  hessian.makeCompressed();
  return hessian;
}

math::RealField3 do_precondition_4th(MassSpring& sys, math::RealField3 const& x) {
  math::RealField1 weight = sys.mass_;
  math::RealField3 result = math::RealField3::Zero(x.rows(), x.cols());

  // Inertia term
  math::RealField3 y = 2 * sys.position_ - sys.last_position_;
  auto y_accessor = math::make_accessor(y);
  auto x_accessor = math::make_accessor(x);
  auto result_accessor = math::make_accessor(result);
  for (auto [i, yi] : math::enumerate(y_accessor)) {
    yi += sys.dt_ * sys.dt_ * sys.external_force_.col(i) / sys.mass_(i);
    auto xi = x_accessor(i);
    auto mass_i = sys.mass_(0, i);
    auto diff = xi - yi;
    result_accessor(i) = mass_i * diff;
  }

  weight /= sys.dt_ * sys.dt_;
  result /= sys.dt_ * sys.dt_;

  // Potential term
  auto springs_accessor = math::make_accessor(sys.springs_);
  for (auto [i, ij] : math::enumerate(springs_accessor)) {
    auto [a, b] = math::unpack(ij.eval());
    auto xa = x_accessor(a), xb = x_accessor(b);
    auto diff = xa - xb;
    auto length = diff.squaredNorm();
    auto original_length = sys.original_length_(i);

    auto l2 = length - original_length * original_length;
    auto stiffness = sys.stiffness_(i);
    math::RealVector3 grad_a = stiffness * l2 * diff;  // on a

    // math::RealVector<6> grad_full;
    // grad_full.block<3, 1>(0, 0) = grad_a;
    // grad_full.block<3, 1>(3, 0) = -grad_a;
    // math::RealMatrix<6, 6> hessian
    //     = local_hessian_4th(xa, xb, stiffness, stiffness, original_length);
    // math::RealVector<6> result_full = hessian.ldlt().solve(grad_full);
    // result_accessor(a) += stiffness * result_full.block<3, 1>(0, 0);
    // result_accessor(b) += stiffness * result_full.block<3, 1>(3, 0);

    result_accessor(a) += grad_a;
    result_accessor(b) -= grad_a;
    weight(0, a) += stiffness;
    weight(0, b) += stiffness;
  }

  for (auto [i, u] : math::enumerate(result_accessor)) {
    u /= weight(0, i);
  }

  for (auto i : sys.fixed_vertices_) {
    result_accessor(i).setZero();
  }

  return result;
}

void run_once(MassSpring& system) {
  optim::OptProblem prob;
  prob.SetEnergy([&system](const optim::Variable& x) {
    if (use_4th_model) {
      return energy_4th(system, x.reshaped(3, system.position_.cols()));
    } else {
      return energy_quad(system, x.reshaped(3, system.position_.cols()));
    }
  });
  prob.SetGrad([&system](const optim::Variable& x) {
    if (use_4th_model) {
      auto grad = gradient_4th(system, x.reshaped(3, system.position_.cols()));
      return grad.reshaped();
    } else {
      auto grad = gradient_quad(system, x.reshaped(3, system.position_.cols()));
      return grad.reshaped();
    }
  });
  prob.SetHessian([&system](const optim::Variable& x) {
    if (use_4th_model) {
      return global_hessian_4th(system, x.reshaped(3, system.position_.cols()));
    } else {
      return global_hessian_quad(system, x.reshaped(3, system.position_.cols()));
    }
  });
  prob.SetConvergeGrad([&system](const optim::Variable& /*x*/, const optim::Variable& g) {
    Real inf_norm = g.cwiseAbs().maxCoeff();
    gradient_norm.push_back(inf_norm);
    AX_INFO("max|g|={:12.6e}", inf_norm);
    return inf_norm;
  });

  optim::Optimizer_Newton optimizer;
  // TOOOO small. body force = 1 and we expect 1% error, and the gradient norm is:
  // 9.8 * mass * dt^2 * 0.01, and we assume mass equiv to 1
  optimizer.SetTolGrad(0.001);
  optimizer.SetTolVar(1e-6);
  optimizer.SetOptions({{"verbose", 1}, {"strategy", "PolakRibiereClamped"}});

  static std::once_flag of;
  std::call_once(of, [&optimizer]() {
    AX_INFO("Optimizer options: {}", optimizer.GetOptions());
  });
  math::RealField3 y = 2 * system.position_ - system.last_position_;
  auto y_accessor = math::make_accessor(y);
  for (auto [i, yi] : math::enumerate(y_accessor)) {
    yi += system.dt_ * system.dt_ * system.external_force_.col(i) / system.mass_(i);
  }

  for (auto i : system.fixed_vertices_) {
    y.col(i) = system.position_.col(i);
  }

  if (better_initial_guess) {
    y = y - do_precondition_4th(system, y);
  }

  auto result = optimizer.Optimize(prob, y.reshaped());

  if (result.converged_) {
    memcpy(system.next_position_.data(), result.x_opt_.data(),
           system.next_position_.size() * sizeof(Real));
    AX_INFO("Optimization converged in {} iterations, grad={}, var={}", result.n_iter_,
            result.converged_grad_, result.converged_var_);
  } else {
    AX_CRITICAL("Optimization failed: {}", result.err_msg_);
  }
}

math::RealMatrix<3, 2> prox(math::RealVector3 const& x0, math::RealVector3 const& x1,
                            math::RealMatrix<3, 2> const& u, Real stiffness, Real original_length,
                            Real rho, bool is_x0_fixed, bool is_x1_fixed) {
  // Solves for:
  //   minimize_z 1/2 k (|z0 - z1| - l)^2 + 1/2 rho ||x-z+u||^2
  // Take the first order condition:
  //        k (|z0 - z1| - l) (z0 - z1)/s + rho (z0 - (x0 + u0)) = 0   <<<< (eq. 1)
  //        k (|z0 - z1| - l) (z1 - z0)/s + rho (z1 - (x1 + u1)) = 0   <<<< (eq. 2)
  // We can have a close form solution. (Use s to represent |z0 - z1|)
  // 1. sum eqs: center point
  //        z0 + z1 = x0 + x1 + u0 + u1
  // 2. difference
  //        [2 k (s - l) + rho s] (z0 - z1)/|z0 - z1| = rho ((x0 + u0) - (x1 + u1))
  //                                                  = rho xu
  //    and (z0 - z1)/|z0 - z1| is a unit vector, therefore
  //        2k (s - l) + rho s = rho |xu|
  //    we now have |z0 - z1| = (rho |xu| + 2k l) / (rho + 2k)
  // 3. the direction:
  //        z0 - z1 // xu
  //    and if [2 k (s - l) + rho s] > 0,
  //        i.e. (2k + rho) s > 2k l
  //        i.e. rho |xu| + 2kl > 2k l
  //        z0 - z1 = xu * s
  //    otherwise, the negative.
  math::RealMatrix<3, 2> z;
  auto u0 = u.col(0), u1 = u.col(1);

  // if (!is_x0_fixed && !is_x1_fixed) {
    math::RealVector3 xu = x0 - x1 + u0 - u1;
    Real strength = (rho * xu.norm() + 2 * stiffness * original_length) / (2 * stiffness + rho);
    math::RealVector3 direction = xu.normalized();
    math::RealVector3 z0_z1;

    z0_z1 = direction * strength;
    math::RealVector3 center = 0.5 * (x0 + x1 + u0 + u1);
    z.col(0) = center + 0.5 * z0_z1;
    z.col(1) = center - 0.5 * z0_z1;
  // } else if (is_x0_fixed && !is_x1_fixed) {
  //   z.col(0) = x0;
  //   // We can eliminate most things.
  //   //    k (|z0 - z1| - l) (z1 - z0)/s + rho (x1 - z1 + u1) = 0   <<<< (eq. 2)
  //   // is what we need to solve, but we know z0 = x0.
  //   //    k (|x0 - z1| - l) (x0 - z1)/s + rho (x1 - z1 + u1) = 0
  //   // => k (|x0 - z1| - l) (x0 - z1)/s + rho (x0 - z1) = rho (x0 - x1 - u1)
  //   // => k (s - l) + rho s = rho |...|
  //   // => s = (rho |...| + kl) / (k + rho)
  //   // and z1 = x0 + (x0 - x1 - u1) * s / |x0 - x1 - u1|
  //   math::RealVector3 xu = x0 - x1 - u1;
  //   Real strength = (rho * xu.norm() + stiffness * original_length) / (stiffness + rho);
  //   math::RealVector3 direction = xu.normalized();
  //   z.col(1) = x0 - direction * strength;
  // } else if (is_x1_fixed && !is_x0_fixed) {
  //   // similar
  //   z.col(1) = x1;
  //   math::RealVector3 xu = x1 - x0 - u0;
  //   Real strength = (rho * xu.norm() + stiffness * original_length) / (stiffness + rho);
  //   math::RealVector3 direction = xu.normalized();
  //   z.col(0) = x1 - direction * strength;
  // } else {
  //   z.col(0) = x0;
  //   z.col(1) = x1;
  // }
  return z;
}

void run_once_by_admm(MassSpring& system) {
  math::RealField3 y = 2 * system.position_ - system.last_position_;
  for (Index i = 0; i < system.position_.cols(); ++i) {
    y.col(i) += system.dt_ * system.dt_ * system.external_force_.col(i) / system.mass_(i);
  }
  for (auto i : system.fixed_vertices_) {
    y.col(i) = system.position_.col(i);
  }

  // The ADMM algorithms writes:
  //    arg min_x f(x) + g(x)  ==>  arg min_x f(x) + sum_i g(z_i) such that z_i = S_i x
  // We have three steps as following:
  // 1. Local step, use x to update z variable.
  //    z_i <- prox_{g_i}(S_i x + u_i)
  // 2. Global step, use z to update x variable.
  //    x <- prox_{f}(sum_i (S_i^T (z_i - u_i)))
  // 3. update the dual variable u.
  //    u_i <- u + S_i x - z_i
  //
  // There is a very important question: how do we initialize each term?
  // A common strategy is, use
  //  x <- x0.
  //  z <- S_i x.
  //  u <- 0.
  // Is this necessary?

  math::RealField3 x = y;
  auto x_accessor = math::make_accessor(x);
  auto spring = math::make_accessor(system.springs_);
  Index num_vertices = x.cols();
  Index num_springs = system.springs_.cols();
  static std::vector<math::RealMatrix<3, 2>> u(num_springs);
  static bool is_first_run = true;
  static math::RealField1 weights;
  if (weights.size() < 1) {
    weights = system.mass_;
    for (auto [si, ij] : math::enumerate(spring)) {
      weights[ij(0)] += system.stiffness_[si] * (system.dt_ * system.dt_);
      weights[ij(1)] += system.stiffness_[si] * (system.dt_ * system.dt_);
    }
  }

  if (is_first_run || !incremental) {
    for (auto& ui : u) {
      ui.setZero();
    }
    is_first_run = false;
  } else {
    math::RealField3 delta_x_weighted(3, num_vertices);
    for (auto i : utils::range<Index>(num_vertices)) {
      Real wi_m = system.mass_[i];
      delta_x_weighted.col(i) = (x_accessor(i) - system.position_.col(i)) * wi_m / weights[i];
    }
    for (auto [si, ij] : math::enumerate(spring)) {
      auto [i, j] = math::unpack(ij.eval());
      u[si].col(0) += delta_x_weighted.col(i);
      u[si].col(1) += delta_x_weighted.col(j);
      if (system.is_fixed_[i]) {
        u[si].col(0).setZero();
      }
      if (system.is_fixed_[j]) {
        u[si].col(1).setZero();
      }
    }
  }

  std::vector<math::RealMatrix<3, 2>> z(num_springs);
  math::RealField3& x_next = system.next_position_;
  x_next = y;
  auto x_next_accessor = math::make_accessor(x_next);

  math::RealField3 dx = x_next;
  auto dx_accessor = math::make_accessor(dx);

  for (auto iteration : utils::range(max_iteration)) {
    // Upd z.
    x_next.setZero();
    dx.setZero();
    for (auto [si, ij] : math::enumerate(spring)) {
      auto [i, j] = math::unpack(ij.eval());
      math::RealVector3 xi = x_accessor(i), xj = x_accessor(j);
      Real k = system.stiffness_[si];
      Real rho = k;
      z[si] = prox(xi, xj, u[si], k, system.original_length_[si], rho, system.is_fixed_[i],
                   system.is_fixed_[j]);

      // if (abs(z[si].col(0).z() - xi.col(0).z()) > 1e-10) {
      //   AX_INFO("!!! {} {}", si, i);
      //   prox(xi, xj, u[si], k, system.original_length_[si], rho, system.is_fixed_[i],
      //        system.is_fixed_[j]);
      // }
      // if (abs(z[si].col(1).z() - xj.col(0).z()) > 1e-10) {
      //   AX_INFO("!!! {} {}", si, j);
      //   prox(xi, xj, u[si], k, system.original_length_[si], rho, system.is_fixed_[i],
      //        system.is_fixed_[j]);
      // }
      math::RealMatrix<3, 2> zu = z[si] - u[si];
      // x_next_accessor(i) += (zu.col(0)) * rho * (system.dt_ * system.dt_);
      // x_next_accessor(j) += (zu.col(1)) * rho * (system.dt_ * system.dt_);
      dx_accessor(i) += (zu.col(0) - x_accessor(i)) * rho * (system.dt_ * system.dt_);
      dx_accessor(j) += (zu.col(1) - x_accessor(j)) * rho * (system.dt_ * system.dt_);
    }

    // Upd x.
    for (auto i : utils::range<Index>(num_vertices)) {
      Real wi_m = system.mass_[i];
      // Analytic solution, does not conserve the momentum.
      dx_accessor(i) += (y.col(i) - x_accessor(i)) * wi_m; // i.e. -dt^2 S' Rho S (x - z + u)
      x_next_accessor(i) += x_accessor(i) + dx_accessor(i) / weights[i];

      // ADI: Implicit inertia term
      // x_next_accessor(i) = 0.5 * (x_accessor(i) + y.col(i) + dx_accessor(i) / wi_m);
    }
    for (auto id : system.fixed_vertices_) {
      x_next_accessor(id) = system.position_.col(id);
    }
    x.swap(x_next);

    Real max_du_norm = 0;
    // Update the dual variable
    for (auto [si, ij] : math::enumerate(spring)) {
      auto [i, j] = math::unpack(ij.eval());
      math::RealMatrix<3, 2> du;
      du.col(0) = x_accessor(i) - z[si].col(0);
      du.col(1) = x_accessor(j) - z[si].col(1);
      max_du_norm = std::max(max_du_norm, math::norm(du, math::linf));
      u[si] += du;
    }

    // Compute the gradient norm:
    auto g = gradient_quad(system, x);
    Real inf_norm = g.cwiseAbs().maxCoeff();
    gradient_norm.push_back(inf_norm);

    AX_INFO("Iteration {:3}: |delta u|={:12.6e} max|g|={:12.6e}", iteration, max_du_norm, inf_norm);
  }
}

void relax(math::RealVector3& x0, math::RealVector3& x1, Real& lagrangian_multiplier,
           Real stiffness, Real original_length, Real rho, Real dt, bool is_x0_fixed,
           bool is_x1_fixed, Real x0_mass, Real x1_mass) {
  // spring energy:
  //    1
  //   --- k (|x0 - x1| - l)^2
  //    2
  //
  // XPBD model energy: U(x) = 1/(2 alpha) c(x)^2
  // We mark
  //   alpha  = (k dt^2)^-1
  //   c(x)   = |x0 - x1| - l
  //   lambda = - c(x) / alpha
  //
  // Update scheme is Gauss Seidel.
  // 1. Update lagrangian multiplier by:
  //                     -c - alpha lambda
  //       lambda <-  ------------------------
  //                   ||grad||_{1/M} + alpha
  //    where grad is the gradient of c(x).
  // 2. Update the position by:
  //       Delta x = M^-1 (grad c(x)).T lambda

  math::RealVector3 diff = x0 - x1;
  Real c = diff.norm() - original_length;
  Real alpha = 1 / (stiffness * dt * dt);
  math::RealVector3 grad_c = diff.normalized();  // partial to x0
  // We noticed that, the gradient of c(x) is grad c(x) = (x0 - x1) / |x0 - x1|
  // Therefore ||grad_{0, 1}|| is always 1.
  //           ||grad||_{1/M} is always sqrt(1/m0 + 1/m1) = sqrt(m0 + m1 / m0m1)
  Real sqrt_inv_mass = math::sqrt(1 / x0_mass + 1 / x1_mass);
  lagrangian_multiplier = -(c + alpha * lagrangian_multiplier) / (sqrt_inv_mass + alpha);
  math::RealVector3 dx0 = grad_c * lagrangian_multiplier / x0_mass;
  x0 += dx0;
  x1 -= dx0;
}

void run_once_xpbd(MassSpring& system) {
  math::RealField3 y = 2 * system.position_ - system.last_position_;
  for (Index i = 0; i < system.position_.cols(); ++i) {
    y.col(i) += system.dt_ * system.dt_ * system.external_force_.col(i) / system.mass_(i);
  }
  for (auto i : system.fixed_vertices_) {
    y.col(i) = system.position_.col(i);
  }
  math::RealField3 x = y;
  auto x_accessor = math::make_accessor(x);
  Index num_vertices = x.cols();
  Index num_springs = system.springs_.cols();

  math::RealField3& x_next = system.next_position_;
  x_next = y;
  auto x_next_accessor = math::make_accessor(x_next);
  auto spring = math::make_accessor(system.springs_);
  std::vector<Real> lag(num_springs);
  Real dt = system.dt_;

  for (auto iteration : utils::range(max_iteration)) {
    for (auto [si, ij] : math::enumerate(spring)) {
      auto [i, j] = math::unpack(ij.eval());
      math::RealVector3 xi = x_next_accessor(i), xj = x_next_accessor(j);
      Real k = system.stiffness_[si];
      Real original_length = system.original_length_(si);
      relax(xi, xj, lag[si], k, original_length, k, dt, system.is_fixed_[i], system.is_fixed_[j],
            system.mass_(i), system.mass_(j));
      x_next_accessor(i) = xi;
      x_next_accessor(j) = xj;
    }

    for (auto i : system.fixed_vertices_) {
      x_next_accessor(i) = x_accessor(i);
    }

    // calculate the gradient:
    auto g = gradient_quad(system, x);
    Real inf_norm = g.cwiseAbs().maxCoeff();
    AX_INFO("Iteration {:3}: max|g|={:12.6e}", iteration, inf_norm);
  }
}

MassSpring& ensure_mass_spring() {
  if (auto* p = try_get_resource<MassSpring>()) {
    return *p;
  } else {
    auto& ms = add_resource<MassSpring>();
    gl::Lines grid = gl::prim::Grid().Draw();
    ms.position_ = grid.vertices_ * 0.01;
    ms.springs_ = grid.indices_;
    ms.original_length_.resize(ms.springs_.cols());
    ms.stiffness_.setConstant(1, ms.springs_.cols(), 10000);
    ms.mass_.setConstant(1, ms.position_.cols(), 1);
    for (Index i = 0; i < ms.springs_.cols(); ++i) {
      auto [a, b] = math::unpack(ms.springs_.col(i).eval());
      ms.original_length_(i) = (ms.position_.col(a) - ms.position_.col(b)).norm();
    }
    ms.next_position_ = ms.last_position_ = ms.position_;
    ms.external_force_ = math::RealField3::Zero(3, ms.position_.cols());
    ms.external_force_.row(1).setConstant(-9.8);
    ms.dt_ = 1e-2;
    ms.is_fixed_.resize(ms.position_.cols(), false);
    for (Index i = 0; i < sqrt(ms.position_.cols()); ++i) {
      ms.fixed_vertices_.push_back(i);
      ms.is_fixed_[i] = true;
    }

    return ms;
  }
}

void ui_callback(gl::UiRenderEvent) {
  bool need_run = false;
  static bool running = false;
  static bool use_admm = false;

  static const char* methods[] = {"pcg", "admm", "xpbd"};
  static int selected_method = 0;

  if (ImGui::Begin("Simulator")) {
    need_run |= ImGui::Button("Run once");
    ImGui::Checkbox("Running", &running);
    need_run |= running;
    if (ImGui::BeginCombo("methods", methods[selected_method])) {
      for (int i = 0; i < 3; ++i) {
        bool is_selected = selected_method == i;
        if (ImGui::Selectable(methods[i], is_selected)) {
          selected_method = i;
        }

        if (is_selected) {
          ImGui::SetItemDefaultFocus();
        }
      }
      ImGui::EndCombo();
    }

    ImGui::Checkbox("Better Initial Guess", &better_initial_guess);
    ImGui::Checkbox("Incremental", &incremental);
    ImGui::InputInt("Max Iteration", &max_iteration);
  }
  ImGui::End();

  if (need_run) {
    auto& ms = ensure_mass_spring();
    auto time_start = utils::now();
    gradient_norm.clear();
    if (selected_method == 0) {
      run_once(ms);
    } else if (selected_method == 1) {
      run_once_by_admm(ms);
    } else {
      run_once_xpbd(ms);
    }
    auto time_end = utils::now();
    auto elapsed = (time_end - time_start);
    for (auto& gn : gradient_norm) {
      gn = math::log(gn + 1e-19);
    }
    Real l2 = grad_l2_norm(ms, ms.next_position_);
    Real linf = grad_inf_norm(ms, ms.next_position_);
    AX_INFO("Time elapsed: {} ms, l2={:12.6e}, linf={:12.6e}",
            std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(), l2, linf);

    // Update rendering.
    static Entity ent = create_entity();
    if (!has_component<gl::Lines>(ent)) {
      add_component<gl::Lines>(ent);
    }

    patch_component<gl::Lines>(ent, [&](gl::Lines& lines) {
      lines.vertices_ = ms.next_position_;
      lines.indices_ = ms.springs_;
      lines.colors_.setConstant(4, lines.vertices_.cols(), 1);

      math::RealField3 grad = gradient_quad(ms, ms.next_position_);  // I know this is not correct.
      // math::RealVectorX residual_on_vertices = (grad.colwise().norm().array() + 1e-19).log();
      math::RealVectorX residual_on_vertices = grad.cwiseAbs().colwise().maxCoeff();
      gl::Colormap cm(0, residual_on_vertices.maxCoeff());
      lines.colors_.topRows(3) = cm(residual_on_vertices);
    });

    ms.position_.swap(ms.last_position_);
    ms.next_position_.swap(ms.position_);
  }

  if (ImGui::Begin("Plotting")) {
    if (ImPlot::BeginPlot("Converge status")) {
      ImPlot::PlotLine("log 10 of Gradient Inf norm", gradient_norm.data(), gradient_norm.size(),
                       1.0f);
      ImPlot::EndPlot();
    }
  }
  ImGui::End();
}

int main(int argc, char** argv) {
  po::add_option({
    po::make_option<bool>(
        "no-quad", "Use 4th order model elasticity instead of quadratic for efficiency.", "false"),
    po::make_option<bool>("better-init-guess", "Use a better initial guess for the optimization.",
                          "false"),
    po::make_option<int>("max-iteration", "Maximum iteration for the optimization.", "10"),
    po::make_option<bool>("incremental", "Use incremental potential energy.", "false"),
  });
  gl::init(argc, argv);
  use_4th_model = po::get_parse_result()["no-quad"].as<bool>();
  better_initial_guess = po::get_parse_result()["better-init-guess"].as<bool>();
  incremental = po::get_parse_result()["incremental"].as<bool>();
  max_iteration = po::get_parse_result()["max-iteration"].as<int>();
  AX_INFO("Use 4th order model: {}", use_4th_model);

  connect<gl::UiRenderEvent, &ui_callback>();
  return gl::enter_main_loop();
}
