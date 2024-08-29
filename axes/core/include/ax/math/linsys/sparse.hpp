#pragma once

#include "ax/math/linsys/preconditioner.hpp"
#include "ax/math/sparse.hpp"
#include "ax/utils/enum_refl.hpp"
#include "common.hpp"

namespace ax::math {

AX_DEFINE_ENUM_CLASS(SparseSolverKind,
                     // Eigen solvers
                     kLDLT, kLLT, kLU, kQR, kConjugateGradient, kLeastSquaresConjugateGradient,
                     kBiCGSTAB,
                     // Cholmod
                     kCholmod
);

class SparseSolverBase : public utils::Tunable {
public:
  SparseSolverBase();
  virtual ~SparseSolverBase() = default;
  static std::unique_ptr<SparseSolverBase> Create(SparseSolverKind kind);
  virtual SparseSolverKind GetKind() const = 0;

  /************************* SECT: Setup the solver *************************/
  /**
   * @brief Set the preconditioner, although some solvers may not use it
   *
   * @param preconditioner
   */
  void SetPreconditioner(std::unique_ptr<PreconditionerBase> preconditioner);

  /**
   * @brief Set the Problem object
   *
   * @param problem
   * @return
   */
  SparseSolverBase& SetProblem(std::shared_ptr<LinsysProblem_Sparse> problem);
  SparseSolverBase& SetProblem(RealSparseMatrix const& A);
  SparseSolverBase& SetProblem(RealSparseMatrix&& A);

  /**
   * @brief Get the Problem object
   */
  std::shared_ptr<LinsysProblem_Sparse> const& GetProblem() const;

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
  virtual LinsysSolveResult Solve(RealMatrixX const& b, RealMatrixX const& x0 = {}) = 0;

  AX_FORCE_INLINE LinsysSolveResult operator()(RealMatrixX const& b, RealMatrixX const& x0 = {}) {
    return Solve(b, x0);
  }

protected:
  std::shared_ptr<LinsysProblem_Sparse> cached_problem_{nullptr};
  std::unique_ptr<PreconditionerBase> preconditioner_{nullptr};
};

}  // namespace ax::math
