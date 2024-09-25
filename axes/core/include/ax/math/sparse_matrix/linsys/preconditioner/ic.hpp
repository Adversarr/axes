#include "ax/math/sparse_matrix/linsys/preconditioner.hpp"

namespace ax::math {

class GeneralSparsePreconditioner_IncompleteCholesky : public GeneralSparsePreconditionerBase {
public:
  GeneralSparsePreconditioner_IncompleteCholesky();
  ~GeneralSparsePreconditioner_IncompleteCholesky() noexcept;

  void AnalyzePattern() override;
  void Factorize() override;

  void Solve(ConstRealBufferView b, RealBufferView x) const override;

  GeneralPreconditionerKind GetKind() const override;

  struct Impl;

private:
  std::unique_ptr<Impl> pimpl_;
};

}  // namespace ax::math