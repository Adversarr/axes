#pragma once
#include "ax/math/linsys/sparse.hpp"

namespace ax::math {

AX_DEFINE_ENUM_CLASS(CholmodSupernodalKind,
                     Auto,        // automatic
                     Simplicial,  // use simplicial LDLT
                     Supernodal   // use supernodal LLT, numerical issue may occur
);

class SparseSolver_Cholmod final : public HostSparseSolverBase {
public:
  SparseSolver_Cholmod();
  ~SparseSolver_Cholmod() override;
  HostSparseSolverKind GetKind() const override;

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

  CholmodSupernodalKind supernodal_kind_ = CholmodSupernodalKind::Auto;
  bool verbose_{false};
  bool check_{false};
};

}  // namespace ax::math
