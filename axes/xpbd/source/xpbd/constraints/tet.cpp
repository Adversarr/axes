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

template <idx dim> ConstraintSolution<dim> Constraint_Tetra<dim>::SolveDistributed() {
  idx const nV = this->GetNumConstrainedVertices();
  idx const nC = this->GetNumConstraints();
  ConstraintSolution<dim> solution(nV);
  std::vector<math::matr<dim, dim + 1>> consensus;
  consensus.resize(nC);
  for (auto i : utils::iota(nC)) {
    auto const ij = this->constraint_mapping_[i];
    for (auto j : utils::iota(dim + 1)) {
      consensus[i].col(j) = this->constrained_vertices_position_[ij[j]];
    }
  }

  real sqr_res = 0;
  for (idx i : utils::iota(nC)) {
    // relax dual
    auto& cur = dual_[i];
    auto const& u = gap_[i];
    auto& R = rotation_[i];
    auto const& z = consensus[i];
    auto const& y = x0_[i];
    real rho = this->rho_[i];
    real k = stiffness_[i];
    real sqr_res_i;
    for (idx iter = 0; iter < substeps_; ++iter) {
      relax_R<dim>(cur, y, R);
      sqr_res_i = relax_dual<dim>(cur, y, R, z, u, rho, k);
    }
    sqr_res += sqr_res_i;
  }

  for (idx i : utils::iota(nC)) {
    auto const& ij = this->constraint_mapping_[i];
    auto const& dual = dual_[i];
    auto const& gap = gap_[i];
    real rho = this->rho_[i];
    for (auto j : utils::iota(dim + 1)) {
      idx vi = ij[j];
      solution.weighted_position_.col(vi) += (dual.col(j) + gap.col(j)) * rho;
      solution.weights_[vi] += rho;
    }
  }

  solution.sqr_dual_residual_ = sqr_res;
  return solution;
}

template <idx dim> void Constraint_Tetra<dim>::BeginStep() {
  idx nC = this->GetNumConstraints();
  this->UpdatePositionConsensus();
  dual_.resize(nC);
  for (idx i = 0; i < nC; ++i) {
    auto const ij = this->constraint_mapping_[i];
    for (idx j = 0; j <= dim; ++j) {
      dual_[i].col(j) = this->constrained_vertices_position_[ij[j]];
    }
  }
  // this->rho_ = stiffness_;
  for (idx i = 0; i < nC; ++i) {
    this->rho_[i] = stiffness_[i];
  }
  this->rho_global_ = 1;
  gap_.resize(nC);
  for (auto& g : gap_) {
    g.setZero();
  }
}

template <idx dim> real Constraint_Tetra<dim>::UpdateDuality() {
  idx nC = this->GetNumConstraints();
  auto const& g = ensure_server<dim>();
  real sqr_prim_res = 0;
  for (idx i = 0; i < nC; ++i) {
    auto const ij = this->constraint_mapping_[i];
    auto const& local = dual_[i];
    auto& gap = gap_[i];
    for (idx j = 0; j <= dim; ++j) {
      idx vi = ij[j];
      auto const& zi = g.vertices_.col(vi);
      math::vecr<dim> primal_residual = local.col(j) - zi;
      gap.col(j) += primal_residual;
      sqr_prim_res += math::norm2(primal_residual);
    }
  }
  return sqr_prim_res;
}

template <idx dim> void Constraint_Tetra<dim>::EndStep() {}

template <idx dim> void Constraint_Tetra<dim>::UpdateRhoConsensus(real scale) {
  // this->rho_ *= scale;
  for (auto& r : this->rho_) r *= scale;
  this->rho_global_ *= scale;
  for (auto& g : gap_) {
    g /= scale;
  }
}

template <idx dim>
void Constraint_Tetra<dim>::SetTetrahedrons(math::fieldi<dim + 1> const& tetrahedrons,
                                            math::field1r const& stiff) {
  idx const nC = tetrahedrons.cols();
  std::set<idx> associated;
  for (auto i : utils::iota(nC)) {
    auto const& ij = tetrahedrons.col(i);
    for (auto j : utils::iota(dim + 1)) {
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

  // this->constraint_mapping_.resize(dim + 1, nC);
  // this->constrained_vertices_position_.resize(dim, associated.size());
  for (auto i : utils::iota(nC)) {
    auto const& ij = tetrahedrons.col(i);
    this->constraint_mapping_.emplace_back(ij);
    for (auto j : utils::iota(dim + 1)) {
      idx l = global_to_local[ij(j)];
      this->constraint_mapping_[i][j] = l;
    }
    this->rho_.push_back(stiff[i]);
  }
  stiffness_ = stiff;

  this->UpdatePositionConsensus();
  x0_.resize(nC);
  rotation_.resize(nC);
  auto const& local = this->constrained_vertices_position_;
  for (idx i : utils::iota(nC)) {
    auto const& ij = this->constraint_mapping_[i];
    auto& x0 = x0_[i];
    for (idx j : utils::iota(dim + 1)) {
      x0.col(j) = local[ij[j]];
    }
    center_<dim>(x0);
  }
}

template class Constraint_Tetra<2>;
template class Constraint_Tetra<3>;

}  // namespace ax::xpbd