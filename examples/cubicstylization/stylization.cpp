#include "stylization.hpp"

#include "ax/geometry/normal.hpp"
#include "ax/math/decomp/svd/import_eigen.hpp"
#include "ax/math/decomp/svd/remove_rotation.hpp"
#include "ax/math/linalg.hpp"
#include "ax/math/linsys/sparse/ConjugateGradient.hpp"
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

void local_step(Problem& prob, idx i) {
  // For i-th...
  math::mat3r& Ri = prob.Ri[i];
  auto const& Di = prob.Di[i];
  auto const& Di0 = prob.Di0[i];
  auto const& ni = prob.ni.col(i);
  auto const& Wi = prob.Wi[i];

  // fix coefficient
  real rho = 1e-3, lambda = 1;
  real tau = 2.0, mu = 10.0;
  vec3r zi, ui;
  zi.setZero();
//  zi = Ri * ni;
  ui.setZero();
  math::mat3xr Dl_mat, Dr_mat;
  Dl_mat.resize(3, (idx)Di.size() + 1);
  Dr_mat.resize(3, (idx)Di.size() + 1);
  Dl_mat.col(Di.size()) = ni;
  for (idx j = 0; j < (idx)Di.size(); ++j) {
    Dl_mat.col(j) = Di0[j];
    Dr_mat.col(j) = Di[j];
  }
  math::matxxr diag = math::zeros(Di.size() + 1, Di.size() + 1);
  for (idx j = 0; j < (idx)Di.size(); ++j) {
    diag(j, j) = Wi[j];
  }
  decomp::JacobiSvd<3, real> svd;
  for (idx it = 0; it < 100; ++it) {
    // Compute Ri.
    math::mat3r Mi = math::zeros<3, 3>();
    Dr_mat.col(Di.size()) = (zi - ui);
    diag(Di.size(), Di.size()) = rho;
    Mi = Dl_mat * diag * Dr_mat.transpose();

    // SVD
    auto result = svd.Solve(Mi);
    auto svd_r = result.value();
    // Ri = V * U.T
    Ri = svd_r.V_ * svd_r.U_.transpose();
    if (Ri.determinant() < 0) {
        svd_r.U_.col(2) *= -1;
        Ri = svd_r.V_ * svd_r.U_.transpose();
    }
    AX_CHECK(Ri.determinant() > 0);
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
  }
}

void Solver::Step(idx steps) {
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

      problem.neighbours[vi].push_back(vj);
      //      problem.neighbours[vj].push_back(vi);

      vec3r di = mesh_.vertices_.col(vj) - mesh_.vertices_.col(vi);
      vec3r dj = mesh_.vertices_.col(vk) - mesh_.vertices_.col(vi);
      real area = norm(cross(di, dj), l2) / 2;
      real cot_weight = std::max(dot(di, dj) / (2 * area), 0.);

      problem.Di[vi].push_back(di);
      problem.Wi[vi].push_back(cot_weight);
      problem.ai[vi] += area;
    }
  }
  problem.Di0 = problem.Di;

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

  LinsysProblem_Sparse linsys;
  linsys.A_ = make_sparse_matrix(3 * n_vert, 3 * n_vert, coef);
  SparseSolver_LDLT ldlt;
  AX_CHECK_OK(ldlt.Analyse(linsys));


  // Do Local global
  for (idx i = 0; i < steps; ++i) {
    // Local Step, solve R, and get z.
    for (idx v = 0; v < n_vert; ++v) {
      // TODO: Use TBB.
      local_step(problem, v);
    }

    auto vert_result = mesh_.vertices_;
    // Strategy 1: Use Linsys
    math::vecxr b = vert_result.reshaped();
    for (idx vi = 0; vi < n_vert; ++vi) {
      for (idx j = 0; j < problem.neighbours[vi].size(); ++ j) {
        idx vj = problem.neighbours[vi][j];
        math::vec3r xi = mesh_.vertices_.col(vi);
        math::vec3r xj = mesh_.vertices_.col(vj);

        // ||xj - xi - Ri (x0j - x0i)||_(xj) = xj - xi - Ri (x0j - x0i)
        auto d0 = problem.Di0[vi][j];
        math::vec3r residual = (problem.Ri[vi] * d0) * problem.Wi[vi][j];
        b.segment<3>(3 * vj) += residual;
        b.segment<3>(3 * vi) -= residual;
      }
    }
    vert_result = ldlt.Solve(b, b)->solution_.reshaped(3, n_vert).eval();
    AX_LOG(INFO) << "Global Iteration " << i << ": dx=" << norm(vert_result - mesh_.vertices_);
    for (idx i = 2; i < n_vert; ++i) {
      mesh_.vertices_.col(i) = vert_result.col(i);
    }
//    mesh_.vertices_ = vert_result;
    // Strategy 2. Jacobi.
//    math::field1r weight(n_vert); weight.setOnes();
//    for (idx vi = 0; vi < n_vert; ++vi) {
//      for (idx j = 0; j < problem.neighbours[vi].size(); ++ j) {
//        idx vj = problem.neighbours[vi][j];
//        math::vec3r xi = mesh_.vertices_.col(vi);
//        math::vec3r xj = mesh_.vertices_.col(vj);
//
//        // ||xj - xi - Ri.T (x0j - x0i)||_(xj) = xj - xi - Ri.T (x0j - x0i) => xi = xj - residual.
//        auto d0 = problem.Di0[vi][j];
//        math::vec3r residual = (xj - problem.Ri[vi].transpose() * d0) * problem.Wi[vi][j];
//        vert_result.col(vi) += residual;
//        weight(0, vi) += problem.Wi[vi][j];
//      }
//    }
//    for (idx i = 2; i < n_vert; ++i) {
//      mesh_.vertices_.col(i) = vert_result.col(i) / weight(0, i);
//    }


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
