#pragma once
#include "ax/math/lattice.hpp"
#include "ax/utils/opt.hpp"
#include "ax/math/linsys/sparse.hpp"

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
template <int dim> class PoissonProblemCellCentered : public utils::Tunable {
public:
  using RealLattice = ax::math::Lattice<dim, Real>;
  using veci = ax::math::IndexVector<dim>;
  static constexpr Real invalid_value = std::numeric_limits<Real>::infinity();
  // Constructor
  PoissonProblemCellCentered(Index n, Real dx);

  // Check if the problem is available
  Status CheckAvailable();

  // Solve the problem
  StatusOr<RealLattice> Solve();

  // If the boundaries are unchanged, we can solve the problem more efficiently
  StatusOr<RealLattice> SolveUnchanged(math::RealVectorX const& source);

  // Domain and boundary conditions
  void SetDomain(math::Lattice<dim, PoissonProblemCellType> const& domain);
  void SetSource(RealLattice const& f);
  void SetBoundaryCondition(math::Lattice<dim, std::array<PoissonProblemBoundaryType, dim>> const& bc_type,
                            math::Lattice<dim, math::RealVector<dim>> const& bc_value);
  void ReportDomain();

  // PDE coefficients
  void SetA(Real a);
  void SetDx(Real dx);

  // Derived from Tunable
  virtual void SetOptions(utils::Options const& option);
  virtual utils::Options GetOptions() const;

private:
  // Problem definition
  Real dx_;
  Index n_;
  Real a_;
  RealLattice f_;

  // Boundary conditions
  math::Lattice<dim, math::RealVector<dim>> bc_value_;
  math::Lattice<dim, std::array<PoissonProblemBoundaryType, dim>> bc_type_;

  // Solution lattice, including ghost cells and outer cells
  RealLattice solution_;
  math::Lattice<dim, PoissonProblemCellType> cell_type_;

  // Helpers for SolverUnchanged.
  math::RealVectorX bc_source_;
  math::Lattice<dim, Index> dof_map_;

  // Solver
  std::unique_ptr<math::SparseSolverBase> sparse_solver_;
  std::string sparse_solver_name_;
};

}  // namespace ax::pde
