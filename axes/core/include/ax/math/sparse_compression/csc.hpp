#pragma once
#include "ax/math/sparse_ext.hpp"

namespace ax::math {

class SparseCompressMatrix_CSC : public SparseCompressMatrixBase {
public:
  void Assign(spmatr const& A) override;
  void AssignSparsePatternUnchanged(spmatr const& A) override;
  void AssignSparsePattern(spmatr const& A) override;
  SparseCompressKind GetKind() const override { return SparseCompressKind::kCsc; }

private:
  std::vector<idx> col_nnz;
};

}  // namespace ax::math
