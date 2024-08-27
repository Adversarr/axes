#pragma once
#include "ax/math/linsys/sparse.hpp"

namespace ax::math {

AX_DEFINE_ENUM_CLASS(CholmodSupernodalKind,
                     kAuto,        // automatic
                     kSimplicial,  // use simplicial LDLT
                     kSupernodal   // use supernodal LLT, numerical issue may occur
);

class SparseSolver_Cholmod final : public SparseSolverBase {
public:
  SparseSolver_Cholmod();
  ~SparseSolver_Cholmod() override;
  SparseSolverKind GetKind() const override;

  void AnalyzePattern() override;
  void Factorize() override;
  LinsysSolveResult Solve(RealMatrixX const& b, RealMatrixX const& x0) override;
  int FactorizeOnce();

  void SetOptions(utils::Options const& opt) override;
  utils::Options GetOptions() const override;

  RealMatrixX Inverse() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;

  CholmodSupernodalKind supernodal_kind_ = CholmodSupernodalKind::kAuto;
  bool verbose_{false};
  bool check_{false};
};

}  // namespace ax::math
