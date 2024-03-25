#pragma once
#include "axes/math/lattice.hpp"
#include "axes/utils/opt.hpp"
#include "axes/math/linsys/sparse.hpp"

namespace ax::pde {

enum class PoissonProblemBoundaryType : int {
  kDirichlet,
  kNeumann,
  kPeriodic,
  kInvalid
};

enum class PoissonProblemCellType : int {
  kInterior,  // Inside the domain,  will solve
  kOuter,     // Out of boundary,    will not solve, and will not be used
  kDirect,    // Directly specified, will not solve, but will be used,
              // e.g. Dirichlet boundary condition at cell center.
};

/**
 * @brief Declares a Poisson problem with constant coefficient. The problem is defined as
 *    -∇·∇ u + a u = f
 * where a is a constant coefficient.
 * @tparam dim
 */
template <idx dim> class PoissonProblemCellCentered : public utils::Tunable {
public:
  using RealLattice = ax::math::Lattice<dim, real>;
  using veci = ax::math::veci<dim>;
  static constexpr real invalid_value = std::numeric_limits<real>::infinity();
  // Constructor
  PoissonProblemCellCentered(idx n, real dx);

  // Check if the problem is available
  Status CheckAvailable();

  // Solve the problem
  StatusOr<RealLattice> Solve();

  // If the boundaries are unchanged, we can solve the problem more efficiently
  StatusOr<RealLattice> SolveUnchanged(math::vecxr const& source);

  // Domain and boundary conditions
  void SetDomain(math::Lattice<dim, PoissonProblemCellType> const& domain);
  void SetSource(RealLattice const& f);
  void SetBoundaryCondition(math::Lattice<dim, std::array<PoissonProblemBoundaryType, dim>> const& bc_type,
                            math::Lattice<dim, math::vecr<dim>> const& bc_value);
  void ReportDomain();

  // PDE coefficients
  void SetA(real a);
  void SetDx(real dx);

  // Derived from Tunable
  virtual Status SetOptions(utils::Opt const& option);
  virtual utils::Opt GetOptions() const;

private:
  // Problem definition
  real dx_;
  idx n_;
  real a_;
  RealLattice f_;

  // Boundary conditions
  math::Lattice<dim, math::vecr<dim>> bc_value_;
  math::Lattice<dim, std::array<PoissonProblemBoundaryType, dim>> bc_type_;

  // Solution lattice, including ghost cells and outer cells
  RealLattice solution_;
  math::Lattice<dim, PoissonProblemCellType> cell_type_;

  // Helpers for SolverUnchanged.
  math::vecxr bc_source_;
  math::Lattice<dim, idx> dof_map_;

  // Solver
  UPtr<math::SparseSolverBase> sparse_solver_;
  std::string sparse_solver_name_;
};

}  // namespace ax::pde