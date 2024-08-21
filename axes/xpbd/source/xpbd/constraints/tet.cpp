#include "ax/xpbd/constraints/tet.hpp"

#include "ax/math/decomp/svd/import_eigen.hpp"
#include "ax/math/decomp/svd/remove_rotation.hpp"
#include "ax/math/linalg.hpp"
#include "ax/utils/ndrange.hpp"

namespace ax::xpbd {
template <typename DA, typename DB, typename DC>
inline void center_(math::MBr<DA>& a, math::MBr<DB>& b, math::MBr<DC>& c) {
  math::RealVector<3> center = (a + b + c) / 3;
  a -= center;
  b -= center;
  c -= center;
}

template <typename DA, typename DB, typename DC, typename DD>
inline void center_(math::MBr<DA>& a, math::MBr<DB>& b, math::MBr<DC>& c, math::MBr<DD>& d) {
  math::RealVector<3> center = (a + b + c + d) * 0.25;
  a -= center;
  b -= center;
  c -= center;
  d -= center;
}

template <int dim> inline math::RealVector<dim> center_(math::RealMatrix<dim, dim + 1>& inout) {
  math::RealVector<dim> center = inout.rowwise().sum() / (dim + 1);
  inout.colwise() -= center;
  return center;
}

template <int dim>
inline Real relax_dual(math::RealMatrix<dim, dim + 1>& cur, math::RealMatrix<dim, dim + 1> const& y,
                       math::RealMatrix<dim, dim> const& R, math::RealMatrix<dim, dim + 1> const& z,
                       math::RealMatrix<dim, dim + 1> const& u, Real rho, Real k) {
  math::RealMatrix<dim, dim + 1> zc = z - u;
  math::RealVector<dim> center = center_<dim>(zc);
  math::RealMatrix<dim, dim + 1> cx = (k * R * y + rho * zc) / (rho + k);
  cx.colwise() += center;
  Real sqr_res = (cx - cur).squaredNorm();
  cur = cx;
  return sqr_res;
}

template <int dim> inline void relax_R(math::RealMatrix<dim, dim + 1> const& x,
                                       math::RealMatrix<dim, dim + 1> const& y, math::RealMatrix<dim, dim>& R) {
  // assume y is zero-centered
  auto x_zc = x;
  center_<dim>(x_zc);
  math::RealMatrix<dim, dim> xyt = x_zc * y.transpose();
  // polar decomposition to RS.
  math::decomp::SvdResult<dim> svdr = math::decomp::JacobiSvd<dim, Real>().Solve(xyt);
  // math::decomp::svd_remove_rotation<Real>(svdr);
  R = svdr.U_ * svdr.V_.transpose();
}

template <int dim> inline void relax_R(math::RealMatrix<dim, dim + 1> const& cur,
                                       math::RealMatrix<dim, dim + 1> const& y, math::RealMatrix<dim, dim>& R,
                                       math::RealMatrix<dim, dim + 1> const& z,
                                       math::RealMatrix<dim, dim + 1> const& u, Real rho, Real k) {
  // assume y is zero-centered
  math::RealMatrix<dim, dim + 1> zc = (cur - z + u) + (k / rho) * cur;
  center_<dim>(zc);
  math::RealMatrix<dim, dim> xyt = zc * y.transpose();
  // polar decomposition to RS.
  math::decomp::SvdResult<dim> svdr = math::decomp::JacobiSvd<dim, Real>().Solve(xyt);
  math::decomp::svd_remove_rotation<Real>(svdr);
  R = svdr.U_ * svdr.V_.transpose();
}

ConstraintSolution Constraint_Tetra::SolveDistributed() {
  Index const nV = this->GetNumConstrainedVertices();
  Index const nC = this->GetNumConstraints();
  ConstraintSolution solution(nV);
  std::vector<math::RealMatrix<3, 4>> consensus;
  consensus.resize(nC);
  for (auto i : utils::range(nC)) {
    auto const ij = this->constraint_mapping_[i];
    for (auto j : utils::range(4)) {
      consensus[i].col(j) = this->constrained_vertices_position_[ij[j]];
    }
  }

  Real const dt = ensure_server().dt_;
  Real sqr_res = 0;
  for (Index i : utils::range(nC)) {
    // relax dual
    auto& cur = dual_[i];
    auto const& u = gap_[i];
    auto& R = rotation_[i];
    auto const& z = consensus[i];
    auto const& y = x0_[i];
    Real rho = this->rho_[i];
    Real k = stiffness_[i] * dt * dt;
    Real sqr_res_i;
    for (Index iter = 0; iter < substeps_; ++iter) {
      relax_R<3>(cur, y, R);
      sqr_res_i = relax_dual<3>(cur, y, R, z, u, rho, k);
    }
    sqr_res += sqr_res_i;
  }

  for (Index i : utils::range(nC)) {
    auto const& ij = this->constraint_mapping_[i];
    auto const& dual = dual_[i];
    auto const& gap = gap_[i];
    Real rho = this->rho_[i];
    for (auto j : utils::range(4)) {
      Index vi = ij[j];
      solution.weighted_position_.col(vi) += (dual.col(j) + gap.col(j)) * rho;
      solution.weights_[vi] += rho;
    }
  }

  solution.sqr_dual_residual_ = sqr_res;
  return solution;
}

void Constraint_Tetra::BeginStep() {
  Index nC = this->GetNumConstraints();
  this->UpdatePositionConsensus();
  dual_.resize(nC);
  for (Index i = 0; i < nC; ++i) {
    auto const ij = this->constraint_mapping_[i];
    for (Index j = 0; j <= 3; ++j) {
      dual_[i].col(j) = this->constrained_vertices_position_[ij[j]];
    }
  }
  // this->rho_ = stiffness_;
  const Real dt = ensure_server().dt_;
  for (Index i = 0; i < nC; ++i) {
    this->rho_[i] = stiffness_[i] * dt * dt;
  }
  this->rho_global_ = 1;
  gap_.resize(nC);
  for (auto& g : gap_) {
    g.setZero();
  }
}

Real Constraint_Tetra::UpdateDuality() {
  Index nC = this->GetNumConstraints();
  Real sqr_prim_res = 0;
  for (Index i = 0; i < nC; ++i) {
    auto const ij = this->constraint_mapping_[i];
    auto const& local = dual_[i];
    auto& gap = gap_[i];
    for (Index j = 0; j <= 3; ++j) {
      Index vi = ij[j];
      auto const& zi = constrained_vertices_position_[vi];
      math::RealVector3 primal_residual = local.col(j) - zi;
      gap.col(j) += primal_residual;
      sqr_prim_res += math::norm2(primal_residual);
    }
  }
  return sqr_prim_res;
}

void Constraint_Tetra::EndStep() {}

void Constraint_Tetra::UpdateRhoConsensus(Real scale) {
  // this->rho_ *= scale;
  for (auto& r : this->rho_) r *= scale;
  this->rho_global_ *= scale;
  for (auto& g : gap_) {
    g /= scale;
  }
}

void Constraint_Tetra::SetTetrahedrons(math::IndexField<4> const& tetrahedrons,
                                       math::RealField1 const& stiff) {
  Index const nC = tetrahedrons.cols();
  std::set<Index> associated;
  for (auto i : utils::range(nC)) {
    auto const& ij = tetrahedrons.col(i);
    for (auto j : utils::range(4)) {
      associated.insert(ij(j));
    }
  }
  this->constrained_vertices_ids_.resize(associated.size());
  std::map<Index, Index> global_to_local;
  Index l = 0;
  for (auto g : associated) {
    this->constrained_vertices_ids_[l] = g;
    global_to_local[g] = l;
    l++;
  }
  Real const dt = ensure_server().dt_;
  for (auto i : utils::range(nC)) {
    auto const& ij = tetrahedrons.col(i);
    this->constraint_mapping_.emplace_back(ij);
    for (auto j : utils::range(4)) {
      Index l = global_to_local.at(ij(j));
      this->constraint_mapping_[i][j] = l;
    }
    this->rho_.push_back(stiff[i] * dt * dt);
  }
  stiffness_ = stiff;

  this->UpdatePositionConsensus();
  x0_.resize(nC);
  rotation_.resize(nC);
  auto const& local = this->constrained_vertices_position_;
  for (Index i : utils::range(nC)) {
    auto const& ij = this->constraint_mapping_[i];
    auto& x0 = x0_[i];
    for (Index j : utils::range(4)) {
      x0.col(j) = local[ij[j]];
    }
    center_<3>(x0);
  }
}

}  // namespace ax::xpbd