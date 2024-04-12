#include "stylization.hpp"
#include "ax/math/decomp/svd/import_eigen.hpp"
#include "ax/math/linalg.hpp"
using namespace ax::math;

struct Problem {
  List<math::mat3r> Ri;
  List<math::mat3r> Di;
  List<math::mat3r> Di0;
  List<math::vec3r> ni;
  List<math::vec3r> zi;
  math::field3r Wi;
  real rho = 1.0;
  real lambda = 1.0;
  math::field1r area;
  math::field3r u;
};

void local_step(Problem& prob) {
  math::decomp::JacobiSvd<3, real> svd;
  for (size_t i = 0; i < prob.Ri.size(); ++i) {
    math::mat3r Di = prob.Di[i];
    math::mat3r Di0 = prob.Di0[i];
    math::matr<3, 4> Di_hat_ni;
    math::matr<3, 4> Di0_zu;
    Di_hat_ni << Di, prob.ni[i];
    Di0_zu << Di0, (prob.zi[i] - prob.u.col(i));
    math::matr<4, 4> diag;
    diag.setZero();
    diag(0, 0) = prob.Wi(0, i);
    diag(1, 1) = prob.Wi(1, i);
    diag(2, 2) = prob.Wi(2, i);
    diag(3, 3) = prob.rho;
    math::mat3r A = Di_hat_ni * diag * Di0_zu.transpose();
    auto status = svd.Solve(A);
    AX_CHECK_OK(status.status());
    prob.Ri[i] = status->V_ * status->U_.transpose();
  }
}

vec3r shrinkage(const vec3r& v, real kappa) {
  vec3r z;
  real norm_v = norm(v, l2);
  real coef = std::max<real>(0.0, 1 - kappa / norm_v);
  return coef * v;
}

void global_step(Problem& prob) {
  // Shrinkage step: kappa = lambda a_i / rho, z <- shrinkage(u + R n, kappa)
  for (size_t i = 0; i < prob.zi.size(); ++i) {
    real kappa = prob.lambda * prob.area[i] / prob.rho;
    prob.zi[i] = shrinkage(prob.u.col(i) + prob.Ri[i] * prob.ni[i], kappa);
  }
}

void Solver::Step(idx steps) {
  Problem problem;
  // Initialize R to Identity:
  idx n_vert = mesh_.vertices_.cols();
  idx n_triangle = mesh_.indices_.cols();
  problem.Ri.resize(n_triangle, math::eye<3>());
  problem.Di.reserve(n_triangle);
  problem.ni.reserve(n_triangle);
  problem.area.resize(1, n_triangle);
  problem.Wi.resize(3, n_triangle);
  for (idx t = 0;  t < n_triangle; ++t) {
    idx i = mesh_.indices_(0, t), j = mesh_.indices_(1, t), k = mesh_.indices_(2, t);
    math::vec3r xi = mesh_.vertices_.col(i);
    math::vec3r xj = mesh_.vertices_.col(j);
    math::vec3r xk = mesh_.vertices_.col(k);
    math::mat3r D;
    D << xi - xj, xj - xk, xk - xi;
    math::vec3r n = cross(xi - xj, xj - xk);
    real a = norm(n, l2);
    problem.Di.push_back(D);
    problem.ni.emplace_back(n / a);
    problem.area[t] = a;

    // cotangent weights:
    problem.Wi(0, t) =-math::dot(xj - xk, xk - xi) / a;
    problem.Wi(1, t) =-math::dot(xk - xi, xi - xj) / a;
    problem.Wi(2, t) =-math::dot(xi - xj, xj - xk) / a;
  }
  problem.zi = problem.ni;
  problem.Di0 = problem.Di;
  problem.u.setZero(3, n_triangle);

  // Do Local global
  for (idx i = 0; i < steps; ++i) {
    local_step(problem); // compute Ri
    // Update Di:
    for (idx t = 0; t < n_triangle; ++t) {
      problem.Di[t] = problem.Di0[t] * problem.Ri[t];
    }
    global_step(problem); // compute z
    // Update U -> u + R n - z
    for (idx t = 0; t < n_triangle; ++t) {
      for (idx j = 0; j < 3; ++j) {
        idx v = mesh_.indices_(j, t);
        problem.u(j, v) = problem.u(j, v) + problem.Ri[t].col(j).dot(problem.ni[t]) - problem.zi[t](j);
      }
    }
  }

  
}
