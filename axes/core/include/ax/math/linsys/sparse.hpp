#pragma once

#include "ax/math/linsys/preconditioner.hpp"
#include "ax/math/sparse.hpp"
#include "ax/utils/enum_refl.hpp"
#include "common.hpp"

namespace ax::math {

BOOST_DEFINE_FIXED_ENUM_CLASS(SparseSolverKind, idx, kLDLT, kLLT, kLU, kQR, kConjugateGradient,
                              kLeastSquaresConjugateGradient, kBiCGSTAB, kCholmod);

class SparseSolverBase : public utils::Tunable {
public:
  SparseSolverBase();
  virtual ~SparseSolverBase() = default;
  static UPtr<SparseSolverBase> Create(SparseSolverKind kind);
  virtual SparseSolverKind GetKind() const = 0;

  /************************* SECT: Setup the solver *************************/
  /**
   * @brief Set the preconditioner, although some solvers may not use it
   *
   * @param preconditioner
   */
  void SetPreconditioner(UPtr<PreconditionerBase> preconditioner);

  /**
   * @brief Set the Problem object
   *
   * @param problem
   * @return
   */
  SparseSolverBase& SetProblem(SPtr<LinsysProblem_Sparse> problem);
  SparseSolverBase& SetProblem(spmatr const& A);
  SparseSolverBase& SetProblem(spmatr&& A);

  /**
   * @brief Get the Problem object
   */
  SPtr<LinsysProblem_Sparse> const& GetProblem() const;

  /************************* SECT: Solving the linear system *************************/
  /**
   * @brief Analyze the pattern of the matrix
   */
  virtual void AnalyzePattern() = 0;
  /**
   * @brief Factorize the matrix
   */
  virtual void Factorize() = 0;
  /**
   * @brief Compute the factorization of the matrix, equivalant to AnalyzePattern then Factorize
   */
  void Compute();

  /**
   * @brief Solve the linear system with optional initial guess
   *
   * @param b the load vector
   * @param x0 initial guess
   * @return
   */
  virtual LinsysSolveResult Solve(math::vecxr const& b, math::vecxr const& x0 = {}) = 0;
  AX_FORCE_INLINE LinsysSolveResult operator()(math::vecxr const& b, math::vecxr const& x0 = {}) {
    return Solve(b, x0);
  }

protected:
  SPtr<LinsysProblem_Sparse> cached_problem_{nullptr};
  UPtr<PreconditionerBase> preconditioner_{nullptr};
};

}  // namespace ax::math
