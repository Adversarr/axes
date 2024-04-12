#include "stylization.hpp"

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
  vec3r z;
  real norm_v = norm(v, l2);
  real coef = std::max<real>(0.0, 1 - kappa / norm_v);
  return coef * v;
}

void local_step(Problem& prob, idx i) {
  // For i-th...
  math::mat3r& Ri = prob.Ri[i];
  auto const& Di = prob.Di[i];
  auto const& Di0 = prob.Di0[i];
  auto const& ni = prob.ni.col(i);
  auto const& Wi = prob.Wi[i];

  // fix coefficient
  real rho = 1, lambda = 1e-2;

  vec3r zi, ui;
  zi.setZero();
  ui.setZero();
  math::mat3xr Dl_mat, Dr_mat;
  Dl_mat.resize(3, Di.size() + 1);
  Dr_mat.resize(3, Di.size() + 1);
  for (idx j = 0; j < (idx)Di.size(); ++j) {
    Dl_mat.col(j) = Di[j];
    Dr_mat.col(j) = Di0[j];
  }
  for (idx it = 0; it < 3; ++it) {
    // Compute Ri.
    math::mat3r Mi = math::zeros<3, 3>();
    Dl_mat.col(Di.size()) = ni;
    Dr_mat.col(Di.size()) = (zi - ui);
    math::matxxr diag = math::zeros(Di.size() + 1, Di.size() + 1);
    for (idx j = 0; j < (idx)Di.size(); ++j) {
      diag(j, j) = Wi[j];
    }
    diag(Di.size(), Di.size()) = rho;
    Mi = Dl_mat * diag * Dr_mat.transpose();

    // SVD
    auto svd = Mi.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    auto U = svd.matrixU();
    auto V = svd.matrixV();
    // Ri = V * U.T
    Ri = V * U.transpose();

    if (svd.info() != Eigen::Success) {
      AX_LOG(ERROR) << "i=" << it <<  "SVD failed: " << Mi;
      std::cout << Dl_mat << std::endl;
      std::cout << diag << std::endl;
      std::cout << Dr_mat << std::endl;
    }
    // z, u
    zi = shrinkage(Ri * ni + ui, lambda * prob.ai[i] / rho);
    ui = ui + Ri * ni - zi;
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
  std::cout << problem.ni << std::endl;

  for (idx t = 0; t < n_triangle; ++t) {
    for (idx j = 0; j < 3; ++j) {
      idx vi = mesh_.indices_(j, t);
      idx vj = mesh_.indices_((j + 1) % 3, t);
      idx vk = mesh_.indices_((j + 2) % 3, t);

      problem.neighbours[vi].push_back(vj);
      problem.neighbours[vj].push_back(vi);

      vec3r di = mesh_.vertices_.col(vj) - mesh_.vertices_.col(vi);
      vec3r dj = mesh_.vertices_.col(vk) - mesh_.vertices_.col(vi);
      real area = norm(cross(di, dj), l2) / 2;
      real cot_weight = std::max(dot(di, dj) / (2 * area), 0.);

      problem.Di[vi].push_back(di);
      problem.Wi[vi].push_back(cot_weight);
      problem.ai[vi] += area;

      std::cout << cot_weight << std::endl;
    }
  }
  problem.Di0 = problem.Di;


  // build global matrix
  sp_coeff_list coeff_list;
  for (idx t = 0; t < n_triangle; ++t) {
    for (idx j = 0; j < 3; ++j) {
      idx vi = mesh_.indices_(j, t);
      idx vj = mesh_.indices_((j + 1) % 3, t);
      idx vk = mesh_.indices_((j + 2) % 3, t);
      vec3r di = mesh_.vertices_.col(vj) - mesh_.vertices_.col(vi);
      vec3r dj = mesh_.vertices_.col(vk) - mesh_.vertices_.col(vi);
      real area = norm(cross(di, dj), l2) / 2;
      real cot_weight = std::max(dot(di, dj) / (2 * area), 0.);

      // xi - xj - R.T dij = 0
      for (size_t k = 0; k < 3; ++k) {
        coeff_list.push_back({vi, vi, cot_weight});
        coeff_list.push_back({vj, vj, cot_weight});
        coeff_list.push_back({vi, vj, -cot_weight});
        coeff_list.push_back({vj, vi, -cot_weight});
      }
    }
  }

  // Do Local global
  for (idx i = 0; i < steps; ++i) {
    // Local Step, solve R, and get z.
    // for (idx i = 0; i < 10; ++i) {
    //   for (idx v = 0; v < n_vert; ++v) {
    //     local_step(problem, v);
    //   }
    // }

    // Global Step, solve x, and get D.
    vecxr weight(n_vert);
    real mass = 1e4;
    field3r vert_copy = mesh_.vertices_ * mass;
    weight.setConstant(mass);
    for (idx i = 0; i < n_vert; ++i) {
      // xj - xi - R.T dij = 0. use jacobi
      // xi - xj + R.T dij = 0
      for (size_t j = 0; j < problem.neighbours[i].size(); ++j) {
        idx j_idx = problem.neighbours[i][j];
        vec3r di = problem.Di[i][j];
        vec3r Rdi = problem.Ri[i] * di; // Ri (xj - xi)
        vert_copy.col(j_idx) = ((mesh_.vertices_.col(i) + Rdi) * problem.Wi[i][j] + vert_copy.col(j_idx)).eval();
        vert_copy.col(i) = ((mesh_.vertices_.col(j_idx) - Rdi) * problem.Wi[i][j] + vert_copy.col(i)).eval();
        weight[j_idx] += problem.Wi[i][j];
        weight[i] += problem.Wi[i][j];
      }
    }
    for (idx i = 0; i < n_vert; ++i) {
      real w = weight[i];
      vert_copy.col(i) /= w;
      assert(!std::isnan(vert_copy.col(i).sum()));
      assert(!std::isinf(vert_copy.col(i).sum()));
    }
    mesh_.vertices_ = vert_copy;

    std::cout << mesh_.vertices_ << std::endl;

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
    problem.ni = geo::normal_per_vertex(mesh_.vertices_, mesh_.indices_, geo::face_area_avg);
  }
}
