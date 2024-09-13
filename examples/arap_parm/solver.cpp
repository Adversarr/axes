#include "solver.hpp"

#include <igl/boundary_loop.h>
#include <igl/harmonic.h>
#include <igl/map_vertices_to_circle.h>
#include <tbb/parallel_for.h>

#include "ax/core/logging.hpp"
#include "ax/math/decomp/svd.hpp"
#include "ax/math/linsys/sparse/ConjugateGradient.hpp"
#include "ax/math/linsys/sparse/LDLT.hpp"
#include "ax/utils/ndrange.hpp"

// #ifdef __MSVC__
#undef ERROR

// #endif

namespace xx {

using namespace ax;

ParameterizationSolver::ParameterizationSolver(SurfaceMesh const& mesh) {
  problem_.input_mesh_ = mesh;
  problem_.iso_coords_.resize(mesh.indices_.cols());
  problem_.Li_.resize(mesh.indices_.cols());
  problem_.cotangent_weights_.resize(mesh.indices_.cols());
  problem_.param_.resize(2, mesh.vertices_.cols());

  for (Index t = 0; t < mesh.indices_.cols(); ++t) {
    Index i = mesh.indices_(0, t), j = mesh.indices_(1, t), k = mesh.indices_(2, t);

    RealVector3 p0 = mesh.vertices_.col(i), p1 = mesh.vertices_.col(j), p2 = mesh.vertices_.col(k);

    // foreach p0, p1, p2, we need a local coordinate system
    RealVector3 x_axis = math::normalized(p1 - p0);
    RealVector3 z_axis = math::normalized((p1 - p0).cross(p2 - p0));
    RealVector3 y_axis = math::normalized(z_axis.cross(x_axis));
    math::RealMatrix3 rotate;
    rotate << x_axis, y_axis, z_axis;
    IsoCoord iso_coord;
    RealVector3 p10 = (rotate.transpose() * (p1 - p0)), p20 = (rotate.transpose() * (p2 - p0));
    iso_coord.col(0) = p10.topRows<2>();
    iso_coord.col(1) = p20.topRows<2>();
    problem_.iso_coords_[t] = iso_coord;

    // Cotangent = 1/tan
    for (Index I : utils::range(3)) {
      Index J = (I + 1) % 3;
      Index K = (I + 2) % 3;
      auto p0 = mesh.vertices_.col(mesh.indices_(I, t));
      auto p1 = mesh.vertices_.col(mesh.indices_(J, t));
      auto p2 = mesh.vertices_.col(mesh.indices_(K, t));
      RealVector3 e1 = p1 - p2;
      RealVector3 e2 = p0 - p2;
      Real cot = math::dot(e1, e2) / math::norm(math::cross(e1, e2));
      problem_.cotangent_weights_[t](I) = cot;
    }
  }

  // We initialize the parameterization to Harmonic Map
  Eigen::VectorXi bnd;
  Eigen::MatrixXd V = mesh.vertices_.transpose();
  Eigen::MatrixXi F = mesh.indices_.transpose().cast<int>();
  Eigen::MatrixXd V_uv;
  igl::boundary_loop(F, bnd);
  Eigen::MatrixXd bnd_uv;
  igl::map_vertices_to_circle(V, bnd, bnd_uv);
  igl::harmonic(V, F, bnd, bnd_uv, 1, V_uv);
  AX_THROW_IF_FALSE(V_uv.rows() == V.rows() && V_uv.cols() == 2, "Invalid tutte embedding");
  for (Index i : utils::range(V_uv.rows())) {
    problem_.param_(0, i) = V_uv(i, 0);
    problem_.param_(1, i) = V_uv(i, 1);
  }

  global_solver_ = std::make_unique<math::SparseSolver_ConjugateGradient>();
  {
    Index n_triangle = problem_.input_mesh_.indices_.cols(),
          n_vertex = problem_.input_mesh_.vertices_.cols();
    // Step1: Establish the Global Linear System:
    math::RealSparseCOO coeff_list;
    coeff_list.reserve(n_vertex * 2 + n_triangle * 24);
    for (Index t : utils::range(problem_.input_mesh_.indices_.cols())) {
      for (Index i : utils::range(3)) {
        Index vi = problem_.input_mesh_.indices_(i, t);
        Index vj = problem_.input_mesh_.indices_((i + 1) % 3, t);
        Real cot_t_i = problem_.cotangent_weights_[t](i);
        for (int dim : utils::range(2)) {
          coeff_list.push_back({vi * 2 + dim, vj * 2 + dim, -cot_t_i});
          coeff_list.push_back({vj * 2 + dim, vi * 2 + dim, -cot_t_i});
          coeff_list.push_back({vi * 2 + dim, vi * 2 + dim, cot_t_i});
          coeff_list.push_back({vj * 2 + dim, vj * 2 + dim, cot_t_i});
        }
      }
    }
    for (Index v : utils::range(n_vertex)) {
      coeff_list.push_back({v * 2, v * 2, shift_});
      coeff_list.push_back({v * 2 + 1, v * 2 + 1, shift_});
    }
    global_problem_.A_ = math::make_sparse_matrix(2 * n_vertex, 2 * n_vertex, coeff_list);
    // AX_LOG(INFO) << problem.A_.toDense().determinant();
  }
  global_solver_->SetProblem(global_problem_.A_).Compute();
}

void ParameterizationSolver::SetLocalSolver(std::unique_ptr<LocalSolverBase> solver) {
  local_solver_ = std::move(solver);
}

void ParameterizationSolver::SetGlobalSolver(std::unique_ptr<math::SparseSolverBase> solver) {
  global_solver_ = std::move(solver);
  global_solver_->SetProblem(global_problem_.A_).Compute();
}

SurfaceMesh ParameterizationSolver::Optimal() {
  SurfaceMesh mesh = problem_.input_mesh_;
  for (Index i : utils::range(problem_.param_.cols())) {
    mesh.vertices_.block<2, 1>(0, i) = problem_.param_.col(i);
    mesh.vertices_.col(i).z() = 0;
  }
  return mesh;
}

void ParameterizationSolver::Solve(Index max_iter) {
  bool converged = false;
  Index n_triangle = problem_.input_mesh_.indices_.cols(),
        n_vertex = problem_.input_mesh_.vertices_.cols();
  // if (!local_solver_) {
  //   return utils::FailedPreconditionError("Local solver not set");
  // }
  AX_THROW_IF_NULL(local_solver_, "Local solver not set");

  math::RealVectorX last_global_optimal(n_vertex * 2);
  for (Index i : utils::range(n_vertex)) {
    last_global_optimal(i * 2) = problem_.param_(0, i);
    last_global_optimal(i * 2 + 1) = problem_.param_(1, i);
  }

  Index n_iter = 0;
  do {
    // Do local step:
    problem_.Li_ = local_solver_->Optimal(problem_);

    // Do global step:
    math::RealVectorX rhs = last_global_optimal * shift_;
    for (Index t : utils::range(n_triangle)) {
      math::RealMatrix<2, 3> local_coord;
      local_coord << math::zeros<2>(), problem_.iso_coords_[t];
      for (Index i : utils::range(3)) {
        Index vi = problem_.input_mesh_.indices_(i, t);
        Index vj = problem_.input_mesh_.indices_((i + 1) % 3, t);
        RealVector2 xi = local_coord.col(i);
        RealVector2 xj = local_coord.col((i + 1) % 3);
        Real cot_t_i = problem_.cotangent_weights_[t](i);
        RealVector2 Lix = problem_.Li_[t] * (xi - xj);
        rhs.segment<2>(2 * vi) += cot_t_i * Lix;
        rhs.segment<2>(2 * vj) -= cot_t_i * Lix;
      }
    }

    auto global_step_result = global_solver_->Solve(rhs, last_global_optimal);
    if (!global_step_result.converged_) {
      AX_ERROR("Global solver failed to converge");
      return;
    }
    Real dx2 = 0;
    auto global_optimal = global_step_result.solution_;
    /*AX_LOG(INFO) << "RHS=" << rhs;
    AX_LOG(INFO) << "opt=" << global_optimal;
    AX_LOG(INFO) << "A opt=" << problem.A_ * global_optimal;
    AX_LOG(INFO) << "A rhs=" << problem.A_ * rhs;
    AX_LOG(INFO) << "A=" << std::endl << problem.A_;*/
    for (Index i : utils::range(n_vertex)) {
      dx2 += math::norm(problem_.param_.col(i) - global_optimal.block<2, 1>(i * 2, 0));
      problem_.param_.col(i) = global_optimal.block<2, 1>(i * 2, 0);
    }
    // AX_DLOG(INFO) << "Iter " << n_iter << " dx2 = " << dx2;
    AX_INFO("Iter {} dx2 = {}", n_iter, dx2);
    n_iter++;
    if (dx2 < 1e-6 || n_iter >= max_iter) {
      converged = true;
    } else {
      // AX_DLOG(INFO) << "Update to next iteration" << last_global_optimal;
      last_global_optimal = global_optimal;
    }
  } while (!converged);
}

std::vector<RealMatrix2> ARAP::Optimal(ParameterizationProblem const& problem) {
  std::vector<RealMatrix2> result;
  // ARAP: use SVD decomposition.
  Index n_triangle = problem.iso_coords_.size();
  result.resize(n_triangle);
  tbb::parallel_for<Index>(0, n_triangle, [&](Index i) {
    RealVector2 par0 = problem.param_.col(problem.input_mesh_.indices_(0, i));
    RealVector2 par1 = problem.param_.col(problem.input_mesh_.indices_(1, i));
    RealVector2 par2 = problem.param_.col(problem.input_mesh_.indices_(2, i));
    RealMatrix2 Ui;
    Ui << par1 - par0, par2 - par0;
    RealMatrix2 X = problem.iso_coords_[i];
    RealMatrix2 Li = Ui * X.inverse();
    Eigen::JacobiSVD<RealMatrix2> svd(Li, Eigen::ComputeFullU | Eigen::ComputeFullV);
    RealMatrix2 U = svd.matrixU();
    RealMatrix2 V = svd.matrixV();
    Real detU = U.determinant(), detV = V.determinant();
    if (detU < 0 && detV > 0) {
      U.col(1) *= -1;
    } else if (detU > 0 && detV < 0) {
      V.col(1) *= -1;
    }
    RealMatrix2 R = U * V.transpose();
    result[i] = R;
  });

  return result;
}

std::vector<RealMatrix2> ASAP::Optimal(ParameterizationProblem const& problem) {
  std::vector<RealMatrix2> result;
  // ASAP: use SVD decomposition.
  Index n_triangle = problem.iso_coords_.size();
  result.resize(n_triangle);
  tbb::parallel_for<Index>(0, n_triangle, [&](Index i) {
    RealVector2 par0 = problem.param_.col(problem.input_mesh_.indices_(0, i));
    RealVector2 par1 = problem.param_.col(problem.input_mesh_.indices_(1, i));
    RealVector2 par2 = problem.param_.col(problem.input_mesh_.indices_(2, i));
    RealMatrix2 Ui;
    Ui << par1 - par0, par2 - par0;
    RealMatrix2 X = problem.iso_coords_[i];
    RealMatrix2 Li = Ui * X.inverse();
    Eigen::JacobiSVD<RealMatrix2> svd(Li, Eigen::ComputeFullU | Eigen::ComputeFullV);
    RealMatrix2 U = svd.matrixU();
    RealMatrix2 V = svd.matrixV();
    RealMatrix2 R = U * V.transpose() * 0.5 * svd.singularValues().sum();
    result[i] = R;
  });

  return result;
}

}  // namespace xx