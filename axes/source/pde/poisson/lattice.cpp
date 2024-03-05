#include "axes/pde/poisson/lattice.hpp"

#include <Eigen/SparseLU>

#define is_interior(sub) ((sub).minCoeff() > 0 && ((sub).array() - n_).maxCoeff() <= 0)

namespace ax::pde {

template <idx dim> PoissonProblemOnLattice<dim>::PoissonProblemOnLattice(idx n, real dx) {
  n_ = n;
  dx_ = dx;
  math::veci<dim> shape, actual_shape;
  actual_shape.setConstant(n);
  shape.setConstant(n + 2);

  cell_type_ = math::Lattice<dim, PoissonProblemCellType>(shape);
  cell_type_ = PoissonProblemCellType::kGhost;

  f_ = RealLattice(actual_shape);
  f_ = 0;

  solution_ = RealLattice(shape);
  a_ = b_ = 0;
  c_ = 1;
  bc_value_ = math::StaggeredLattice<dim, real>(actual_shape);
  bc_type_ = math::StaggeredLattice<dim, PoissonProblemBoundaryType>(actual_shape);
}

template <idx dim> void PoissonProblemOnLattice<dim>::SetSource(RealLattice const& f) {
  AX_CHECK((f.Shape().array() == f_.Shape().array()).all())
      << "Source shape mismatch:"
      << "f: " << f.Shape().transpose() << ", desired: " << f_.Shape().transpose();
  f_ = f;
}

template <idx dim> void PoissonProblemOnLattice<dim>::SetDomain(
    math::Lattice<dim, PoissonProblemCellType> const& domain) {
  cell_type_ = PoissonProblemCellType::kOuter;
  for (auto const& sub : domain.Iterate()) {
    AX_DCHECK(domain(sub) != PoissonProblemCellType::kGhost)
        << "Ghost cell is not allowed in user input domain.";
    cell_type_(sub + math::ones<dim, 1, idx>()) = domain(sub);
  }

  // After setup the domain, we need to update the ghost cells.
  for (auto const& sub : cell_type_.Iterate()) {
    if (cell_type_(sub) == PoissonProblemCellType::kInterior) {
      AX_DCHECK(is_interior(sub)) << "Interior cell is not in the interior.";
      for (idx d = 0; d < dim; ++d) {
        for (idx s : {-1, 0, 1}) {
          auto neighbor = sub;
          neighbor(d) += s;
          if (cell_type_(neighbor) == PoissonProblemCellType::kOuter) {
            cell_type_(neighbor) = PoissonProblemCellType::kGhost;
          }
        }
      }
    }
  }
}

char to_char(PoissonProblemCellType t) {
  switch (t) {
    case PoissonProblemCellType::kInterior:
      return 'I';
    case PoissonProblemCellType::kGhost:
      return 'G';
    case PoissonProblemCellType::kOuter:
      return 'O';
    case PoissonProblemCellType::kDirect:
      return 'D';
  }
  AX_CHECK(false) << "Unknown PoissonProblemCellType: " << static_cast<int>(t);
}

template <idx dim> void PoissonProblemOnLattice<dim>::ReportDomain() {
  for (auto const& sub : cell_type_.Iterate()) {
    std::cout << sub.transpose() << ": " << to_char(cell_type_(sub)) << std::endl;
  }
}

template <idx dim> void PoissonProblemOnLattice<dim>::SetBoundaryCondition(
    math::StaggeredLattice<dim, PoissonProblemBoundaryType> const& bc_type,
    math::StaggeredLattice<dim, real> const& bc_value) {
  AX_CHECK((bc_type.Shape().array() == bc_value.Shape().array()).all())
      << "Boundary condition shape mismatch:"
      << "bc_type: " << bc_type.Shape().transpose()
      << ", bc_value: " << bc_value.Shape().transpose();

  AX_CHECK((bc_type.Shape().array() == (bc_type_.Shape()).array()).all())
      << "Boundary condition shape mismatch:"
      << "bc_type: " << bc_type.Shape().transpose() << "desired: " << bc_type_.Shape().transpose();

  bc_type_ = bc_type;
  bc_value_ = bc_value;
}

template <idx dim> void PoissonProblemOnLattice<dim>::SetA(real a) {
  AX_CHECK(a >= 0) << "A should be non-negative.";
  a_ = a;
}

template <idx dim> void PoissonProblemOnLattice<dim>::SetB(real b) { b_ = b; }

template <idx dim> void PoissonProblemOnLattice<dim>::SetC(real c) {
  AX_CHECK(c > 0) << "C should be positive.";
  c_ = c;
}

template <idx dim> Status PoissonProblemOnLattice<dim>::CheckAvailable() {
  // Check if the domain is set
  for (auto const& sub : cell_type_.Iterate()) {
    if (!is_interior(sub)) {
      if (cell_type_(sub) == PoissonProblemCellType::kInterior) {
        AX_LOG(ERROR) << "sub" << to_char(cell_type_(sub)) << ": " << sub.transpose()
                      << ", shape: " << cell_type_.Shape().transpose();
        return utils::InternalError("Enlarged Lattice have interior on the boundaries.");
      }
      continue;
    }

    if (cell_type_(sub) == PoissonProblemCellType::kInterior) {
      for (idx d = 0; d < dim; ++d) {
        for (idx s : {-1, 1}) {
          auto neighbor = sub + s * math::unit<dim, idx>(d);
          if (cell_type_(neighbor) == PoissonProblemCellType::kOuter) {
            AX_LOG(ERROR) << "sub" << to_char(cell_type_(sub)) << ": " << sub.transpose()
                          << ", shape: " << cell_type_.Shape().transpose();
            return utils::InternalError("Interior cell can only have Ghost cell as neighbor.");
          }
        }
      }
    }
  }

  // Check if the boundary condition is set
  for (auto const& [sub, t] : cell_type_.Enumerate()) {
    if (t == PoissonProblemCellType::kDirect) {
      // Directly set the cell value, no need to check the boundary condition.
      continue;
    }

    if (is_interior(sub) && cell_type_(sub) == PoissonProblemCellType::kInterior) {
      for (idx d = 0; d < dim; ++d) {
        for (idx s : {-1, 1}) {
          auto neighbor = sub + s * math::unit<dim, idx>(d);
          math::veci<dim> stagger_sub = sub - math::veci<dim>::Ones();
          if (s == 1) {
            stagger_sub(d) += 1;
          }
          if (cell_type_(neighbor) == PoissonProblemCellType::kGhost) {
            // the face is located between `sub` and `neighbour`.

            // Check if the boundary condition is set
            if (bc_type_(d, stagger_sub) == PoissonProblemBoundaryType::kInvalid) {
              AX_LOG(ERROR) << "sub" << to_char(cell_type_(sub)) << ": " << sub.transpose()
                            << ", neighbour" << to_char(cell_type_(neighbor)) << ": "
                            << neighbor.transpose() << ", boundary type is Invalid";
              return utils::InvalidArgumentError("G-I face should have boundary condition.");
            }
          } else if (cell_type_(neighbor) == PoissonProblemCellType::kInterior) {
            // Check there is no boundary condition set on the interior face.
            if (bc_type_(d, stagger_sub) != PoissonProblemBoundaryType::kInvalid) {
              AX_LOG(ERROR) << "sub" << to_char(cell_type_(sub)) << ": " << sub.transpose()
                            << ", neighbour" << to_char(cell_type_(neighbor)) << ": "
                            << neighbor.transpose() << ", boundary type is not Invalid";
              return utils::InvalidArgumentError("I-I face should not have boundary condition.");
            }
          }
        }
      }
    }
  }
  AX_RETURN_OK();
}

template <idx dim>
StatusOr<typename PoissonProblemOnLattice<dim>::RealLattice> PoissonProblemOnLattice<dim>::Solve()
    const {
  idx dofs = 0;
  for (auto const& t : cell_type_) {
    if (t != PoissonProblemCellType::kOuter) {
      dofs++;
    }
  }

  math::vecxr rhs(dofs);
  math::sp_matxxr A(dofs, dofs);
  math::Lattice<dim, idx> dof_map(cell_type_.Shape());
  dof_map = -1;
  std::map<idx, math::veci<dim>> dof_map_inv;
  idx cnt = 0;
  for (auto const& [sub, t] : cell_type_.Enumerate()) {
    if (t != PoissonProblemCellType::kOuter) {
      dof_map(sub) = cnt;
      dof_map_inv[cnt] = sub;
      cnt += 1;
    }
  }

  // build matrix
  math::sp_coeff_list coef;
  idx cnt_constraint = 0;
  for (auto const& [sub, t] : cell_type_.Enumerate()) {
    if (t == PoissonProblemCellType::kDirect) {
      coef.push_back({cnt_constraint, dof_map(sub), 1});
      rhs[cnt_constraint] = solution_(sub);
      cnt_constraint++;
      continue;
    } else if (t != PoissonProblemCellType::kInterior) {
      continue;
    }
    idx dof = dof_map(sub);
    AX_DCHECK(dof >= 0) << "sub: " << sub.transpose() << ", dof: " << dof;

    // The poisson operator:
    coef.push_back({cnt_constraint, dof, a_ + (2 * dim) * c_ / (dx_ * dx_)});
    for (idx d = 0; d < dim; ++d) {
      for (idx s : {-1, 1}) {
        auto neighbor = sub + s * math::unit<dim, idx>(d);
        auto neighbor_dof = dof_map(neighbor);
        coef.push_back({cnt_constraint, neighbor_dof, s * b_ / dx_ - c_ / (dx_ * dx_)});
      }
    }

    rhs[cnt_constraint] = f_(sub - math::veci<dim>::Ones());
    cnt_constraint += 1;

    for (idx d = 0; d < dim; ++d) {
      for (idx s : {-1, 1}) {
        auto neighbor = sub + s * math::unit<dim, idx>(d);
        math::veci<dim> stagger_sub = sub - math::veci<dim>::Ones();
        if (s == 1) {
          stagger_sub(d) += 1;
        }

        if (cell_type_(neighbor) == PoissonProblemCellType::kGhost) {
          // the face is located between `sub` and `neighbour`.
          idx neighbor_dof = dof_map(neighbor);
          if (bc_type_(d, stagger_sub) == PoissonProblemBoundaryType::kDirichlet) {
            coef.push_back({cnt_constraint, neighbor_dof, 0.5});
            coef.push_back({cnt_constraint, dof, 0.5});
            rhs[cnt_constraint] = bc_value_(d, stagger_sub);
            cnt_constraint++;
          } else if (bc_type_(d, stagger_sub) == PoissonProblemBoundaryType::kNeumann) {
            coef.push_back({cnt_constraint, neighbor_dof, -1});
            coef.push_back({cnt_constraint, dof, 1});
            rhs[cnt_constraint] = bc_value_(d, stagger_sub) * dx_;
            cnt_constraint++;
          }
        }
      }
    }
  }

  if (cnt_constraint > dofs) {
    AX_LOG(ERROR) << "cnt_constraint: " << cnt_constraint << ", dofs: " << dofs;
    return utils::InternalError("Too many constraints.");
  } else if (cnt_constraint < dofs) {
    AX_LOG(ERROR) << "cnt_constraint: " << cnt_constraint << ", dofs: " << dofs;
    return utils::InternalError("Too few constraints.");
  }

  A.resize(dofs, dofs);
  A.setFromTriplets(coef.begin(), coef.end());
  Eigen::SparseLU lu(A);
  if (lu.info() != Eigen::Success) {
    AX_LOG(ERROR) << "Matrix: \n" << A.toDense();
    return utils::InternalError("Decomposition failed.");
  }

  auto sol = lu.solve(rhs);
  RealLattice solution(f_.Shape());
  for (idx i = 0; i < dofs; ++i) {
    auto v = dof_map_inv[i];
    if (is_interior(v)) {
      solution(v - math::veci<dim>::Ones()) = sol[i];
    }
  }

  return solution;
}

template class PoissonProblemOnLattice<2>;
template class PoissonProblemOnLattice<3>;
}  // namespace ax::pde