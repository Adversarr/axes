#include "axes/pde/poisson/lattice.hpp"

#include <Eigen/SparseCholesky>

#define is_interior(sub) ((sub).minCoeff() > 0 && ((sub).array() - n_).maxCoeff() <= 0)

namespace ax::pde {

template <idx dim> PoissonProblemOnLattice<dim>::PoissonProblemOnLattice(idx n, real dx) {
  n_ = n;
  dx_ = dx;
  math::veci<dim> shape, actual_shape;
  actual_shape.setConstant(n);
  shape.setConstant(n + 2);

  cell_type_ = math::Lattice<dim, PoissonProblemCellType>(shape);
  cell_type_ = PoissonProblemCellType::kOuter;

  f_ = RealLattice(actual_shape);
  f_ = 0;

  solution_ = RealLattice(shape);
  a_ = 0;
  b_.setZero();
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
    cell_type_(sub + math::ones<dim, 1, idx>()) = domain(sub);
  }
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

template <idx dim> void PoissonProblemOnLattice<dim>::SetB(math::vecr<dim> const& b) { b_ = b; }

template <idx dim> void PoissonProblemOnLattice<dim>::SetC(real c) {
  AX_CHECK(c > 0) << "C should be positive.";
  c_ = c;
}

template <idx dim> Status PoissonProblemOnLattice<dim>::CheckAvailable() {
  // Check if the domain is set
  for (auto const& sub : cell_type_.Iterate()) {
    if (!is_interior(sub)) {
      if (cell_type_(sub) == PoissonProblemCellType::kInterior ||
          cell_type_(sub) == PoissonProblemCellType::kDirect) {
        AX_LOG(ERROR) << "sub" << to_char(cell_type_(sub)) << ": " << sub.transpose()
                      << ", shape: " << cell_type_.Shape().transpose();
        return utils::InternalError("Enlarged Lattice have interior on the boundaries.");
      }
      continue;
    }
  }

  // Check if the boundary condition is set
  for (auto const& [sub, t] : cell_type_.Enumerate()) {
    if (is_interior(sub) && cell_type_(sub) == PoissonProblemCellType::kInterior) {
      for (idx d = 0; d < dim; ++d) {
        for (idx s : {-1, 1}) {
          auto neighbor = sub + s * math::unit<dim, idx>(d);
          math::veci<dim> stagger_sub = sub - math::veci<dim>::Ones();
          if (s == 1) {
            stagger_sub(d) += 1;
          }
          if (cell_type_(neighbor) == PoissonProblemCellType::kOuter) {
            // Check if the boundary condition is set
            if (bc_type_(d, stagger_sub) == PoissonProblemBoundaryType::kInvalid) {
              AX_LOG(ERROR) << "sub" << to_char(cell_type_(sub)) << ": " << sub.transpose()
                            << ", neighbour" << to_char(cell_type_(neighbor)) << ": "
                            << neighbor.transpose() << ", boundary type is Invalid";
              return utils::InvalidArgumentError("G-I face should have boundary condition.");
            }
          } else if (cell_type_(neighbor) == PoissonProblemCellType::kInterior ||
                     cell_type_(neighbor) == PoissonProblemCellType::kDirect) {
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
  rhs.setZero();
  math::sp_matxxr A(dofs, dofs);
  math::Lattice<dim, idx> dof_map(cell_type_.Shape());
  dof_map = -1;
  std::map<idx, math::veci<dim>> dof_map_inv;
  idx cnt = 0;
  for (auto const& [sub, t] : cell_type_.Enumerate()) {
    if (t == PoissonProblemCellType::kInterior) {
      dof_map(sub) = cnt;
      dof_map_inv[cnt] = sub;
      cnt += 1;
    }
  }

  // build matrix
  math::sp_coeff_list coef;
  coef.reserve(dofs * 5);  // 5 point stencil
  const idx si[2] = {-1, 1};
  const real c_dx_dx = c_ / dx_ / dx_;
  idx cnt_constraint = 0;
  for (auto const& [sub, t] : cell_type_.Enumerate()) {
    idx dof = dof_map(sub);
    if (cell_type_(sub) != PoissonProblemCellType::kInterior) {
      continue;
    }
    // Eqns
    // TODO: Now we use 5Point Stencil, we can make it more general, and accurate.
    real local_center = a_ + (2 * dim) * c_dx_dx;
    real local_rhs = f_(sub - math::veci<dim>::Ones());
    for (idx d = 0; d < dim; ++d) {
      for (int i = 0; i < 2; ++i) {
        idx s = si[i];
        auto neighbor = sub + s * math::unit<dim, idx>(d);
        math::veci<dim> stagger_sub = sub - math::veci<dim>::Ones();
        if (s == 1) {
          stagger_sub(d) += 1;
        }

        if (cell_type_(neighbor) == PoissonProblemCellType::kInterior) {
          coef.push_back({cnt_constraint, dof_map(neighbor), -c_dx_dx});
        } else if (cell_type_(neighbor) == PoissonProblemCellType::kOuter) {
          auto bc_val = bc_value_(d, stagger_sub);
          auto bc_type = bc_type_(d, stagger_sub);
          if (bc_type == PoissonProblemBoundaryType::kDirichlet) {
            AX_DLOG(INFO) << "Dirichlet: " << sub.transpose() << ", " << neighbor.transpose();
            local_center += c_dx_dx;
            local_rhs += c_dx_dx * bc_val * 2;
          } else if (bc_type == PoissonProblemBoundaryType::kNeumann) {
            AX_DLOG(INFO) << "Neumann: " << sub.transpose() << ", " << neighbor.transpose();
            local_center -= c_dx_dx;
            local_rhs += c_dx_dx * bc_val * dx_;
          }
        } else if (cell_type_(neighbor) == PoissonProblemCellType::kDirect) {
          local_rhs += c_dx_dx * solution_(neighbor);
        }
      }
    }

    AX_DLOG(INFO) << "local_center: " << local_center << ", local_rhs: " << local_rhs;
    coef.push_back({cnt_constraint, dof, local_center});
    rhs[cnt_constraint] = local_rhs;
    cnt_constraint += 1;
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
  Eigen::SimplicialLDLT lu(A);
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