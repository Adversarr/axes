#pragma once
#include "axes/math/lattice.hpp"

namespace ax::pde {

enum class PoissonProblemBoundaryType {
  kDirichlet,
  kNeumann,
  kInvalid
};

enum class PoissonProblemCellType {
  kInterior,  // Inside the domain,  will solve
  kGhost,     // On the boundary,    will solve, help to solve the boundary conditions
  kOuter,     // Out of boundary,    will not solve, and will not be used
  kDirect,    // Directly specified, will not solve, but will be used,
              // e.g. Dirichlet boundary condition at cell center.
};

/**
 * @brief Declares a Poisson problem with constant coefficient. The problem is defined as
 *    -∇·(c ∇u) + b ∇·u + a u = f
 * where a, b, c is a constant coefficient.
 * @note c could be a scalar or a lattice.
 * @tparam dim
 */
template <idx dim> class PoissonProblemOnLattice {
public:
  using RealLattice = ax::math::Lattice<dim, real>;
  using veci = ax::math::veci<dim>;
  static constexpr real invalid_value = std::numeric_limits<real>::infinity();

  StatusOr<RealLattice> Solve() const;

  Status CheckAvailable();

  PoissonProblemOnLattice(idx n, real dx);

  void SetDomain(math::Lattice<dim, PoissonProblemCellType> const& domain);

  void SetBoundaryCondition(math::StaggeredLattice<dim, PoissonProblemBoundaryType> const& bc_type,
                            math::StaggeredLattice<dim, real> const& bc_value);

  void ReportDomain();

  void SetA(real a);
  void SetB(real b);
  void SetC(real c);
  void SetSource(RealLattice const& f);

  void SetDx(real dx);


private:
  // Problem definition
  real dx_;
  idx n_;
  real c_;
  real b_;
  real a_;
  RealLattice f_;

  // Boundary conditions
  math::StaggeredLattice<dim, real> bc_value_;
  math::StaggeredLattice<dim, PoissonProblemBoundaryType> bc_type_;

  // Solution lattice, including ghost cells and outer cells
  RealLattice solution_;
  math::Lattice<dim, PoissonProblemCellType> cell_type_;
};

}  // namespace ax::pde