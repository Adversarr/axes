#include "stylization.hpp"

#include <tbb/parallel_for.h>

#include "ax/geometry/normal.hpp"
#include "ax/math/decomp/svd/import_eigen.hpp"
#include "ax/math/linalg.hpp"
#include "ax/math/linsys/sparse/LDLT.hpp"
#include "ax/utils/iota.hpp"
using namespace ax::math;

struct Problem {
  List<math::mat3r> Ri;
  List<List<idx>> neighbours;
  List<List<math::vec3r>> Di;
  List<List<math::vec3r>> Di0;
  math::field3r ni;
  List<real> ai;
  List<List<real>> Wi;
};

vec3r shrinkage(const vec3r& v, real kappa) {
  vec3r tmp1 = v.array() - kappa;
  vec3r tmp2 = -v.array() - kappa;
  vec3r posmax = tmp1.array().max(0);
  vec3r negmax = tmp2.array().max(0);
  return posmax - negmax;
}

void local_step(Problem& prob, idx i, Dijkstra& solver) {
  // For i-th...
  math::mat3r& Ri = prob.Ri[i];
  auto const& Di = prob.Di[i];
  auto const& Di0 = prob.Di0[i];
  auto const& ni = prob.ni.col(i);
  auto const& Wi = prob.Wi[i];

  // fix coefficient
  real rho = solver.rho_, lambda = solver.lambda_;
  real tau = solver.tau_, mu = solver.mu_;
  vec3r zi, ui;
  zi.setZero();
  ui.setZero();
  math::mat3r Mi_without_n = math::zeros<3, 3>();
  for (idx j = 0; j < (idx)Di.size(); ++j) {
    Mi_without_n += Wi[j] * Di0[j] * Di[j].transpose();
  }

  decomp::JacobiSvd<3, real> svd;
  for (idx it = 0; it < 100; ++it) {
    // Compute Ri.
    math::mat3r Mi = Mi_without_n;
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

    real r_norm = (zi - Ri * ni).norm();
    real s_norm = (rho * (zi - zi_last)).norm();
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

void Dijkstra::Step(idx steps) {
  Problem problem;
  idx n_vert = mesh_.vertices_.cols();
  idx n_triangle = mesh_.indices_.cols();
  problem.Ri.resize(n_vert, math::eye<3>());
  problem.neighbours.resize(n_vert);
  problem.Di.resize(n_vert);
  problem.Wi.resize(n_vert);
  problem.ai.resize(n_vert, 0);
  problem.ni = geo::normal_per_vertex(mesh_.vertices_, mesh_.indices_, geo::face_area_avg);
  for (idx t = 0; t < n_triangle; ++t) {
    for (idx j = 0; j < 3; ++j) {
      idx vi = mesh_.indices_(j, t);
      idx vj = mesh_.indices_((j + 1) % 3, t);
      idx vk = mesh_.indices_((j + 2) % 3, t);
      vec3r di = mesh_.vertices_.col(vj) - mesh_.vertices_.col(vi);
      vec3r dj = mesh_.vertices_.col(vk) - mesh_.vertices_.col(vi);
      real area = norm(cross(di, dj), l2) / 2;
      real cot_weight = std::max(dot(di, dj) / (2 * area), 0.);

      problem.neighbours[vi].push_back(vj);
      problem.Di[vi].push_back(di);
      problem.Wi[vi].push_back(cot_weight);
      problem.ai[vi] += area;
    }
  }
  problem.Di0 = problem.Di;
  for (idx i = 0; i < n_vert; ++i) {
    std::cout << problem.ai[i] << std::endl;
  }

  sp_coeff_list coef;
  for (idx i = 0; i < n_vert; ++i) {
    for (idx j = 0; j < problem.neighbours[i].size(); ++j) {
      auto vi = i, vj = problem.neighbours[i][j];
      auto wij = problem.Wi[vi][j];
      for (idx d: utils::iota(3)) {
        coef.push_back({vi * 3 + d, vi * 3 + d, wij});
        coef.push_back({vj * 3 + d, vj * 3 + d, wij});
        coef.push_back({vi * 3 + d, vj * 3 + d, -wij});
        coef.push_back({vj * 3 + d, vi * 3 + d, -wij});
      }
    }
    for (idx d : utils::iota(3)) {
      coef.push_back({i * 3 + d, i * 3 + d, 1});
    }
  }

  SparseSolver_LDLT ldlt;
  ldlt.SetProblem(make_sparse_matrix(3 * n_vert, 3 * n_vert, coef)).Compute();

  // Do Local global
  for (idx i = 0; i < steps; ++i) {
    // Local Step, solve R, and get z.
    tbb::parallel_for(tbb::blocked_range<idx>(0, n_vert, n_vert / 8), [&](const tbb::blocked_range<idx>& r) {
      // TODO: Use TBB.
      for (idx v = r.begin(); v < r.end(); ++v) {
        local_step(problem, v, *this);
      }
    });

    auto vert_result = mesh_.vertices_;
    // Strategy 1: Use Linsys
    math::vecxr b = vert_result.reshaped();
    for (idx vi = 0; vi < n_vert; ++vi) {
      for (idx j = 0; j < problem.neighbours[vi].size(); ++ j) {
        idx vj = problem.neighbours[vi][j];
        // ||xj - xi - Ri (x0j - x0i)||_(xj) = xj - xi - Ri (x0j - x0i)
        auto d0 = problem.Di0[vi][j];
        math::vec3r residual = (problem.Ri[vi] * d0) * problem.Wi[vi][j];
        b.segment<3>(3 * vj) += residual;
        b.segment<3>(3 * vi) -= residual;
      }
    }
    vert_result = ldlt.Solve(b, b).solution_.reshaped(3, n_vert).eval();
    AX_LOG(INFO) << "Global Iteration " << i << ": dx=" << norm(vert_result - mesh_.vertices_);
    mesh_.vertices_ = vert_result;

    // Update Di
    std::for_each(problem.Di.begin(), problem.Di.end(), [](auto& v) { v.clear(); });
    for (idx t = 0; t < n_triangle; ++t) {
      for (idx j = 0; j < 3; ++j) {
        idx vi = mesh_.indices_(j, t);
        idx vj = mesh_.indices_((j + 1) % 3, t);
        vec3r di = mesh_.vertices_.col(vj) - mesh_.vertices_.col(vi);
        problem.Di[vi].push_back(di);
      }
    }
    cached_sequence.push_back(mesh_.vertices_);
  }
}
