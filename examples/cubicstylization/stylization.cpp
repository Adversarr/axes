#include "stylization.hpp"

#include <tbb/parallel_for.h>

#include "ax/geometry/normal.hpp"
#include "ax/math/decomp/svd/import_eigen.hpp"
#include "ax/math/linalg.hpp"
#include "ax/math/linsys/sparse/LDLT.hpp"
#include "ax/utils/ndrange.hpp"
using namespace ax::math;

struct Problem {
  std::vector<math::RealMatrix3> Ri;
  std::vector<std::vector<Index>> neighbours;
  std::vector<std::vector<math::RealVector3>> Di;
  std::vector<std::vector<math::RealVector3>> Di0;
  math::RealField3 ni;
  std::vector<Real> ai;
  std::vector<std::vector<Real>> Wi;
};

RealVector3 shrinkage(const RealVector3& v, Real kappa) {
  RealVector3 tmp1 = v.array() - kappa;
  RealVector3 tmp2 = -v.array() - kappa;
  RealVector3 posmax = tmp1.array().max(0);
  RealVector3 negmax = tmp2.array().max(0);
  return posmax - negmax;
}

void local_step(Problem& prob, Index i, Dijkstra& solver) {
  // For i-th...
  math::RealMatrix3& Ri = prob.Ri[i];
  auto const& Di = prob.Di[i];
  auto const& Di0 = prob.Di0[i];
  auto const& ni = prob.ni.col(i);
  auto const& Wi = prob.Wi[i];

  // fix coefficient
  Real rho = solver.rho_, lambda = solver.lambda_;
  Real tau = solver.tau_, mu = solver.mu_;
  RealVector3 zi, ui;
  zi.setZero();
  ui.setZero();
  math::RealMatrix3 Mi_without_n = math::zeros<3, 3>();
  for (Index j = 0; j < (Index)Di.size(); ++j) {
    Mi_without_n += Wi[j] * Di0[j] * Di[j].transpose();
  }

  decomp::JacobiSvd<3, Real> svd;
  for (Index it = 0; it < 100; ++it) {
    // Compute Ri.
    math::RealMatrix3 Mi = Mi_without_n;
    Mi.noalias() += rho * ni * (zi - ui).transpose();

    // SVD
    auto svd_r =  svd.Solve(Mi);
    // Ri = V * U.T
    auto Rold = Ri;
    Ri = svd_r.V_ * svd_r.U_.transpose();
    if (Ri.determinant() < 0) {
        svd_r.U_.col(2) *= -1;
        Ri = svd_r.V_ * svd_r.U_.transpose();
    }

    // z, u
    auto zi_last = zi;
    zi = shrinkage(Ri * ni + ui, lambda * prob.ai[i] / rho);
    ui = (ui + Ri * ni - zi).eval();

    Real r_norm = (zi - Ri * ni).norm();
    Real s_norm = (rho * (zi - zi_last)).norm();
    if (r_norm > mu * s_norm) {
      rho *= tau;
      ui /= tau;
    } else if (s_norm > mu * r_norm) {
      rho /= tau;
      ui *= tau;
    }

    if (r_norm < 1e-6 && s_norm < 1e-6) {
      break;
    }
  }
}

void Dijkstra::Step(Index steps) {
  Problem problem;
  Index n_vert = mesh_.vertices_.cols();
  Index n_triangle = mesh_.indices_.cols();
  problem.Ri.resize(n_vert, math::eye<3>());
  problem.neighbours.resize(n_vert);
  problem.Di.resize(n_vert);
  problem.Wi.resize(n_vert);
  problem.ai.resize(n_vert, 0);
  problem.ni = geo::normal_per_vertex(mesh_.vertices_, mesh_.indices_, geo::face_area_avg);
  for (Index t = 0; t < n_triangle; ++t) {
    for (Index j = 0; j < 3; ++j) {
      Index vi = mesh_.indices_(j, t);
      Index vj = mesh_.indices_((j + 1) % 3, t);
      Index vk = mesh_.indices_((j + 2) % 3, t);
      RealVector3 di = mesh_.vertices_.col(vj) - mesh_.vertices_.col(vi);
      RealVector3 dj = mesh_.vertices_.col(vk) - mesh_.vertices_.col(vi);
      Real area = norm(cross(di, dj), l2) / 2;
      Real cot_weight = std::max(dot(di, dj) / (2 * area), 0.);

      problem.neighbours[vi].push_back(vj);
      problem.Di[vi].push_back(di);
      problem.Wi[vi].push_back(cot_weight);
      problem.ai[vi] += area;
    }
  }
  problem.Di0 = problem.Di;
  for (Index i = 0; i < n_vert; ++i) {
    std::cout << problem.ai[i] << std::endl;
  }

  SparseCOO coef;
  for (Index i = 0; i < n_vert; ++i) {
    for (Index j = 0; j < problem.neighbours[i].size(); ++j) {
      auto vi = i, vj = problem.neighbours[i][j];
      auto wij = problem.Wi[vi][j];
      for (Index d: utils::range(3)) {
        coef.push_back({vi * 3 + d, vi * 3 + d, wij});
        coef.push_back({vj * 3 + d, vj * 3 + d, wij});
        coef.push_back({vi * 3 + d, vj * 3 + d, -wij});
        coef.push_back({vj * 3 + d, vi * 3 + d, -wij});
      }
    }
    for (Index d : utils::range(3)) {
      coef.push_back({i * 3 + d, i * 3 + d, 1});
    }
  }

  SparseSolver_LDLT ldlt;
  ldlt.SetProblem(make_sparse_matrix(3 * n_vert, 3 * n_vert, coef)).Compute();

  // Do Local global
  for (Index i = 0; i < steps; ++i) {
    // Local Step, solve R, and get z.
    tbb::parallel_for(tbb::blocked_range<Index>(0, n_vert, n_vert / 8), [&](const tbb::blocked_range<Index>& r) {
      // TODO: Use TBB.
      for (Index v = r.begin(); v < r.end(); ++v) {
        local_step(problem, v, *this);
      }
    });

    auto vert_result = mesh_.vertices_;
    // Strategy 1: Use Linsys
    math::RealVectorX b = vert_result.reshaped();
    for (Index vi = 0; vi < n_vert; ++vi) {
      for (Index j = 0; j < problem.neighbours[vi].size(); ++ j) {
        Index vj = problem.neighbours[vi][j];
        // ||xj - xi - Ri (x0j - x0i)||_(xj) = xj - xi - Ri (x0j - x0i)
        auto d0 = problem.Di0[vi][j];
        math::RealVector3 residual = (problem.Ri[vi] * d0) * problem.Wi[vi][j];
        b.segment<3>(3 * vj) += residual;
        b.segment<3>(3 * vi) -= residual;
      }
    }
    vert_result = ldlt.Solve(b, b).solution_.reshaped(3, n_vert).eval();
    AX_LOG(INFO) << "Global Iteration " << i << ": dx=" << norm(vert_result - mesh_.vertices_);
    mesh_.vertices_ = vert_result;

    // Update Di
    std::for_each(problem.Di.begin(), problem.Di.end(), [](auto& v) { v.clear(); });
    for (Index t = 0; t < n_triangle; ++t) {
      for (Index j = 0; j < 3; ++j) {
        Index vi = mesh_.indices_(j, t);
        Index vj = mesh_.indices_((j + 1) % 3, t);
        RealVector3 di = mesh_.vertices_.col(vj) - mesh_.vertices_.col(vi);
        problem.Di[vi].push_back(di);
      }
    }
    cached_sequence.push_back(mesh_.vertices_);
  }
}
