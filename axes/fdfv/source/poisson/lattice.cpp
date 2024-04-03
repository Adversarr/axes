#include "ax/fdfv/poisson/lattice.hpp"

#include <Eigen/IterativeLinearSolvers>
#include <Eigen/SparseCholesky>

#define is_interior(sub) ((sub).minCoeff() >= 0 && ((sub).array() - n_).maxCoeff() < 0)

namespace ax::pde {

template <idx dim> PoissonProblemCellCentered<dim>::PoissonProblemCellCentered(idx n, real dx) {
  n_ = n;
  dx_ = dx;
  math::veci<dim> shape;
  shape.setConstant(n);

  cell_type_ = math::Lattice<dim, PoissonProblemCellType>(shape);
  cell_type_ = PoissonProblemCellType::kOuter;

  f_ = RealLattice(shape);
  f_ = 0;

  solution_ = RealLattice(shape);
  a_ = 0;
  bc_value_.Reshape(shape, math::staggered);
  bc_type_.Reshape(shape, math::staggered);
  dof_map_.Reshape(shape);

  // default sparse solver is ldlt.
  sparse_solver_name_ = "ConjugateGradient";
  sparse_solver_ = math::SparseSolverBase::Create(math::SparseSolverKind::kConjugateGradient);
}

template <idx dim> void PoissonProblemCellCentered<dim>::SetSource(RealLattice const& f) {
  AX_CHECK(math::all(f.Shape().array() == f_.Shape().array()))
      << "Source shape mismatch:"
      << "f: " << f.Shape().transpose() << ", desired: " << f_.Shape().transpose();
  f_ = f;
}

template <idx dim> void PoissonProblemCellCentered<dim>::SetDomain(
    math::Lattice<dim, PoissonProblemCellType> const& domain) {
  cell_type_ = PoissonProblemCellType::kOuter;
  AX_CHECK((domain.Shape().array() == cell_type_.Shape().array()).all())
      << "Domain shape mismatch:"
      << "domain: " << domain.Shape().transpose()
      << ", desired: " << cell_type_.Shape().transpose();
  cell_type_ = domain;
}

char to_char(PoissonProblemCellType t) {
  switch (t) {
    case PoissonProblemCellType::kInterior:
      return 'I';
    case PoissonProblemCellType::kOuter:
      return 'O';
    case PoissonProblemCellType::kDirect:
      return 'D';
  }
  AX_CHECK(false) << "Unknown PoissonProblemCellType: " << static_cast<int>(t);
}

template <idx dim> void PoissonProblemCellCentered<dim>::ReportDomain() {
  for (auto const& sub : cell_type_.Iterate()) {
    std::cout << sub.transpose() << ": " << to_char(cell_type_(sub)) << std::endl;
  }
}

template <idx dim> void PoissonProblemCellCentered<dim>::SetBoundaryCondition(
    math::Lattice<dim, std::array<PoissonProblemBoundaryType, dim>> const& bc_type,
    math::Lattice<dim, math::vecr<dim>> const& bc_value) {
  AX_CHECK((bc_type.Shape().array() == bc_value.Shape().array()).all())
      << "Boundary condition shape mismatch:"
      << "bc_type: " << bc_type.Shape().transpose()
      << ", bc_value: " << bc_value.Shape().transpose();

  AX_CHECK((bc_type.Shape().array() == (bc_type_.Shape()).array()).all())
      << "Boundary condition shape mismatch:"
      << "bc_type: " << bc_type.Shape().transpose() << "desired: " << bc_type_.Shape().transpose();

  AX_CHECK(bc_type.IsStaggered()) << "Boundary condition should be staggered.";
  AX_CHECK(bc_value.IsStaggered()) << "Boundary condition should be staggered.";

  bc_type_ = bc_type;
  bc_value_ = bc_value;
}

template <idx dim> void PoissonProblemCellCentered<dim>::SetA(real a) {
  AX_CHECK(a >= 0) << "A should be non-negative.";
  a_ = a;
}

template <idx dim> void PoissonProblemCellCentered<dim>::SetDx(real dx) {
  AX_CHECK(dx > 0) << "dx should be positive.";
  dx_ = dx;
}

template <idx dim> Status PoissonProblemCellCentered<dim>::CheckAvailable() {
  // Check if the boundary condition is set
  for (auto const& [sub, t] : cell_type_.Enumerate()) {
    if (cell_type_(sub) == PoissonProblemCellType::kInterior) {
      for (idx d = 0; d < dim; ++d) {
        for (idx s : {-1, 1}) {
          auto neighbor = sub + s * math::unit<dim, idx>(d);
          if (!cell_type_.IsSubValid(neighbor, math::cell_center)) {
            continue;
          }

          /**
           *   +-----+
           *   |     |
           *  -1  c  1
           *   |     |
           *   +-----+
           */
          math::veci<dim> stagger_sub = s == -1 ? sub : neighbor;

          if (cell_type_(neighbor) == PoissonProblemCellType::kOuter) {
            // Check if the boundary condition is set
            if (bc_type_(stagger_sub)[d] == PoissonProblemBoundaryType::kInvalid) {
              AX_LOG(ERROR) << "sub" << to_char(cell_type_(sub)) << ": " << sub.transpose()
                            << ", neighbour" << to_char(cell_type_(neighbor)) << ": "
                            << neighbor.transpose() << ", boundary type is Invalid";
              return utils::InvalidArgumentError("G-I face should have boundary condition.");
            }
          } else if (cell_type_(neighbor) == PoissonProblemCellType::kInterior
                     || cell_type_(neighbor) == PoissonProblemCellType::kDirect) {
            // Check there is no boundary condition set on the interior face.
            if (bc_type_(stagger_sub)[d] != PoissonProblemBoundaryType::kInvalid) {
              AX_LOG(ERROR) << "sub" << to_char(cell_type_(sub)) << ": " << sub.transpose()
                            << ", neighbour" << to_char(cell_type_(neighbor)) << ": "
                            << neighbor.transpose() << "BC Sub=" << stagger_sub.transpose()
                            << " d=" << d
                            << ", boundary type is not Invalid." << static_cast<int>(bc_type_(stagger_sub)[d]);
              return utils::InvalidArgumentError("I-I face should not have boundary condition.");
            }
          }
        }
      }
    }
  }
  AX_RETURN_OK();
}

template <idx dim> StatusOr<typename PoissonProblemCellCentered<dim>::RealLattice>
PoissonProblemCellCentered<dim>::Solve() {
  AX_DLOG(INFO) << "PoissonProblemOnLattice Options:" << GetOptions();

  idx dofs = 0;
  for (auto const& t : cell_type_) {
    if (t != PoissonProblemCellType::kOuter) {
      dofs++;
    }
  }

  math::vecxr rhs(dofs);
  rhs.setZero();
  bc_source_.resize(dofs);
  bc_source_.setZero();

  math::sp_matxxr A(dofs, dofs);
  idx cnt = 0;
  for (auto const& [sub, t] : cell_type_.Enumerate()) {
    if (t == PoissonProblemCellType::kInterior) {
      dof_map_(sub) = cnt;
      cnt += 1;
    }
  }
  // build matrix
  math::sp_coeff_list coef;
  coef.reserve(dofs * 5);  // 5 point stencil
  const idx si[2] = {-1, 1};
  const real c = 1.0;
  const real dx_sqr = math::square(dx_);
  idx cnt_constraint = 0;
  for (auto const& [sub, t] : cell_type_.Enumerate()) {
    idx dof = dof_map_(sub);
    if (cell_type_(sub) != PoissonProblemCellType::kInterior) {
      continue;
    }

    // Eqns
    // TODO: Now we use 5Point Stencil, we can make it more general, and accurate.
    real local_center = a_ * dx_sqr + (2 * dim);
    real local_bc_source = 0;
    for (idx d = 0; d < dim; ++d) {
      for (int i = 0; i < 2; ++i) {
        idx s = si[i];
        auto neighbor = sub + s * math::unit<dim, idx>(d);
        math::veci<dim> stagger_sub = sub;
        if (s == 1) {
          stagger_sub(d) += 1;
        }
        auto bc_val = bc_value_(stagger_sub)[d];
        auto bc_type = bc_type_(stagger_sub)[d];
        if (bc_type == PoissonProblemBoundaryType::kDirichlet) {
          AX_DLOG(INFO) << "Dirichlet: " << sub.transpose() << ", " << neighbor.transpose();
          local_center += c;
          local_bc_source += c * bc_val * 2;
        } else if (bc_type == PoissonProblemBoundaryType::kNeumann) {
          AX_DLOG(INFO) << "Neumann: " << sub.transpose() << ", " << neighbor.transpose();
          local_center -= c;
          local_bc_source += c * bc_val * dx_;
        } else if (!is_interior(neighbor)) {
          return utils::InvalidArgumentError("Outer (out-of-bd) cell should not be used.");
        } else if (cell_type_(neighbor) == PoissonProblemCellType::kInterior) {
          coef.push_back({cnt_constraint, dof_map_(neighbor), -c});
        } else if (cell_type_(neighbor) == PoissonProblemCellType::kOuter) {
          return utils::InvalidArgumentError("Outer cell should not be used.");
        } else if (cell_type_(neighbor) == PoissonProblemCellType::kDirect) {
          local_bc_source += c * solution_(neighbor) * dx_sqr;
        }
      }
    }

    AX_DLOG(INFO) << "local_center: " << local_center << ", local_rhs: " << local_bc_source;
    coef.push_back({cnt_constraint, dof, local_center});
    bc_source_[cnt_constraint] = local_bc_source;
    rhs[cnt_constraint] = f_(sub) * dx_sqr;
    cnt_constraint += 1;
  }

  A.resize(dofs, dofs);
  A.setFromTriplets(coef.begin(), coef.end());
  // Eigen::ConjugateGradient<math::sp_matxxr> lu(A);

  // Build the linear system.
  math::LinsysProblem_Sparse sp_problem;
  sp_problem.A_ = std::move(A);
  sp_problem.b_ = rhs + bc_source_;
  auto result = sparse_solver_->SolveProblem(sp_problem);

  if (!result.ok()) {
    return result.status();
  }

  const auto& sol = result.value().solution_;

  RealLattice solution(f_.Shape());
  for (auto const& [sub, t] : cell_type_.Enumerate()) {
    idx dof = dof_map_(sub);
    if (cell_type_(sub) != PoissonProblemCellType::kInterior) {
      continue;
    }
    solution(sub) = sol[dof];
  }
  return solution;
}

template <idx dim> StatusOr<typename PoissonProblemCellCentered<dim>::RealLattice>
PoissonProblemCellCentered<dim>::SolveUnchanged(math::vecxr const& source) {
  if (source.size() != bc_source_.size()) {
    return utils::InvalidArgumentError("Source size mismatch.");
  }
  auto result = sparse_solver_->Solve(bc_source_ + source, math::vecxr::Zero(bc_source_.size()));
  if (!result.ok()) {
    return result.status();
  }

  const auto& sol = result.value().solution_;
  RealLattice solution(f_.Shape());
  for (auto const& [sub, t] : cell_type_.Enumerate()) {
    idx dof = dof_map_(sub);
    if (cell_type_(sub) != PoissonProblemCellType::kInterior) {
      continue;
    }
    solution(sub) = sol[dof];
  }
  return solution;
}

template <idx dim> utils::Opt PoissonProblemCellCentered<dim>::GetOptions() const {
  utils::Opt opt;
  opt["sparse_solver_name"] = sparse_solver_name_;
  opt["sparse_solver_opt"] = sparse_solver_->GetOptions();
  return opt;
}

template <idx dim> Status PoissonProblemCellCentered<dim>::SetOptions(utils::Opt const& option) {
  AX_SYNC_OPT_IF(option, std::string, sparse_solver_name) {
    auto ss = utils::reflect_enum<math::SparseSolverKind>(sparse_solver_name_);
    AX_CHECK(ss) << "Unknown sparse_solver_name: " << sparse_solver_name_;
    sparse_solver_ = math::SparseSolverBase::Create(ss.value());
    AX_RETURN_NOTOK_OR(utils::sync_to_field(*sparse_solver_, option, "sparse_solver_opt"));
  }
  AX_RETURN_OK();
}

template class PoissonProblemCellCentered<2>;
template class PoissonProblemCellCentered<3>;
}  // namespace ax::pde