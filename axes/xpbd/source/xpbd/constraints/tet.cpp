#include "ax/xpbd/constraints/tet.hpp"

#include "ax/math/decomp/svd/import_eigen.hpp"
#include "ax/math/decomp/svd/remove_rotation.hpp"
#include "ax/math/linalg.hpp"
#include "ax/utils/iota.hpp"

namespace ax::xpbd {
template <typename DA, typename DB, typename DC>
inline void center_(math::MBr<DA>& a, math::MBr<DB>& b, math::MBr<DC>& c) {
  math::vecr<3> center = (a + b + c) / 3;
  a -= center;
  b -= center;
  c -= center;
}

template <typename DA, typename DB, typename DC, typename DD>
inline void center_(math::MBr<DA>& a, math::MBr<DB>& b, math::MBr<DC>& c, math::MBr<DD>& d) {
  math::vecr<3> center = (a + b + c + d) * 0.25;
  a -= center;
  b -= center;
  c -= center;
  d -= center;
}

template <idx dim> inline math::vecr<dim> center_(math::matr<dim, dim + 1>& inout) {
  math::vecr<dim> center = inout.rowwise().sum() / (dim + 1);
  inout.colwise() -= center;
  return center;
}

template <idx dim>
inline real relax_dual(math::matr<dim, dim + 1>& cur, math::matr<dim, dim + 1> const& y,
                       math::matr<dim, dim> const& R, math::matr<dim, dim + 1> const& z,
                       math::matr<dim, dim + 1> const& u, real rho, real k) {
  math::matr<dim, dim + 1> zc = z - u;
  math::vecr<dim> center = center_<dim>(zc);
  math::matr<dim, dim + 1> cx = (k * R * y + rho * zc) / (rho + k);
  cx.colwise() += center;
  real sqr_res = (cx - cur).squaredNorm();
  cur = cx;
  return sqr_res;
}

template <idx dim> inline void relax_R(math::matr<dim, dim + 1> const& x,
                                       math::matr<dim, dim + 1> const& y, math::matr<dim, dim>& R) {
  // assume y is zero-centered
  auto x_zc = x;
  center_<dim>(x_zc);
  math::matr<dim, dim> xyt = x_zc * y.transpose();
  // polar decomposition to RS.
  math::decomp::SvdResult<dim> svdr = math::decomp::JacobiSvd<dim, real>().Solve(xyt);
  // math::decomp::svd_remove_rotation<real>(svdr);
  R = svdr.U_ * svdr.V_.transpose();
}

template <idx dim> inline void relax_R(math::matr<dim, dim + 1> const& cur,
                                       math::matr<dim, dim + 1> const& y, math::matr<dim, dim>& R,
                                       math::matr<dim, dim + 1> const& z,
                                       math::matr<dim, dim + 1> const& u, real rho, real k) {
  // assume y is zero-centered
  math::matr<dim, dim + 1> zc = (cur - z + u) + (k / rho) * cur;
  center_<dim>(zc);
  math::matr<dim, dim> xyt = zc * y.transpose();
  // polar decomposition to RS.
  math::decomp::SvdResult<dim> svdr = math::decomp::JacobiSvd<dim, real>().Solve(xyt);
  math::decomp::svd_remove_rotation<real>(svdr);
  R = svdr.U_ * svdr.V_.transpose();
}

ConstraintSolution Constraint_Tetra::SolveDistributed() {
  idx const nV = this->GetNumConstrainedVertices();
  idx const nC = this->GetNumConstraints();
  ConstraintSolution solution(nV);
  std::vector<math::matr<3, 4>> consensus;
  consensus.resize(nC);
  for (auto i : utils::iota(nC)) {
    auto const ij = this->constraint_mapping_[i];
    for (auto j : utils::iota(4)) {
      consensus[i].col(j) = this->constrained_vertices_position_[ij[j]];
    }
  }

  real const dt = ensure_server().dt_;
  real sqr_res = 0;
  for (idx i : utils::iota(nC)) {
    // relax dual
    auto& cur = dual_[i];
    auto const& u = gap_[i];
    auto& R = rotation_[i];
    auto const& z = consensus[i];
    auto const& y = x0_[i];
    real rho = this->rho_[i];
    real k = stiffness_[i] * dt * dt;
    real sqr_res_i;
    for (idx iter = 0; iter < substeps_; ++iter) {
      relax_R<3>(cur, y, R);
      sqr_res_i = relax_dual<3>(cur, y, R, z, u, rho, k);
    }
    sqr_res += sqr_res_i;
  }

  for (idx i : utils::iota(nC)) {
    auto const& ij = this->constraint_mapping_[i];
    auto const& dual = dual_[i];
    auto const& gap = gap_[i];
    real rho = this->rho_[i];
    for (auto j : utils::iota(4)) {
      idx vi = ij[j];
      solution.weighted_position_.col(vi) += (dual.col(j) + gap.col(j)) * rho;
      solution.weights_[vi] += rho;
    }
  }

  solution.sqr_dual_residual_ = sqr_res;
  return solution;
}

void Constraint_Tetra::BeginStep() {
  idx nC = this->GetNumConstraints();
  this->UpdatePositionConsensus();
  dual_.resize(nC);
  for (idx i = 0; i < nC; ++i) {
    auto const ij = this->constraint_mapping_[i];
    for (idx j = 0; j <= 3; ++j) {
      dual_[i].col(j) = this->constrained_vertices_position_[ij[j]];
    }
  }
  // this->rho_ = stiffness_;
  const real dt = ensure_server().dt_;
  for (idx i = 0; i < nC; ++i) {
    this->rho_[i] = stiffness_[i] * dt * dt;
  }
  this->rho_global_ = 1;
  gap_.resize(nC);
  for (auto& g : gap_) {
    g.setZero();
  }
}

real Constraint_Tetra::UpdateDuality() {
  idx nC = this->GetNumConstraints();
  real sqr_prim_res = 0;
  for (idx i = 0; i < nC; ++i) {
    auto const ij = this->constraint_mapping_[i];
    auto const& local = dual_[i];
    auto& gap = gap_[i];
    for (idx j = 0; j <= 3; ++j) {
      idx vi = ij[j];
      auto const& zi = constrained_vertices_position_[vi];
      math::vec3r primal_residual = local.col(j) - zi;
      gap.col(j) += primal_residual;
      sqr_prim_res += math::norm2(primal_residual);
    }
  }
  return sqr_prim_res;
}

void Constraint_Tetra::EndStep() {}

void Constraint_Tetra::UpdateRhoConsensus(real scale) {
  // this->rho_ *= scale;
  for (auto& r : this->rho_) r *= scale;
  this->rho_global_ *= scale;
  for (auto& g : gap_) {
    g /= scale;
  }
}

void Constraint_Tetra::SetTetrahedrons(math::fieldi<4> const& tetrahedrons,
                                       math::field1r const& stiff) {
  idx const nC = tetrahedrons.cols();
  std::set<idx> associated;
  for (auto i : utils::iota(nC)) {
    auto const& ij = tetrahedrons.col(i);
    for (auto j : utils::iota(4)) {
      associated.insert(ij(j));
    }
  }
  this->constrained_vertices_ids_.resize(associated.size());
  std::map<idx, idx> global_to_local;
  idx l = 0;
  for (auto g : associated) {
    this->constrained_vertices_ids_[l] = g;
    global_to_local[g] = l;
    l++;
  }
  real const dt = ensure_server().dt_;
  for (auto i : utils::iota(nC)) {
    auto const& ij = tetrahedrons.col(i);
    this->constraint_mapping_.emplace_back(ij);
    for (auto j : utils::iota(4)) {
      idx l = global_to_local.at(ij(j));
      this->constraint_mapping_[i][j] = l;
    }
    this->rho_.push_back(stiff[i] * dt * dt);
  }
  stiffness_ = stiff;

  this->UpdatePositionConsensus();
  x0_.resize(nC);
  rotation_.resize(nC);
  auto const& local = this->constrained_vertices_position_;
  for (idx i : utils::iota(nC)) {
    auto const& ij = this->constraint_mapping_[i];
    auto& x0 = x0_[i];
    for (idx j : utils::iota(4)) {
      x0.col(j) = local[ij[j]];
    }
    center_<3>(x0);
  }
}

}  // namespace ax::xpbd