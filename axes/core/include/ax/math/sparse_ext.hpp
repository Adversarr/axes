#pragma once
#include "ax/utils/enum_refl.hpp"
#include "sparse.hpp"

namespace ax::math {

BOOST_DEFINE_ENUM_CLASS(SparseCompressKind, kCsr, kCsc, kCoo);

class SparseCompressMatrixBase {
public:
  SparseCompressMatrixBase() = default;
  SparseCompressMatrixBase& operator=(math::spmatr const& A);
  virtual ~SparseCompressMatrixBase() = default;

  static std::unique_ptr<SparseCompressMatrixBase> Create(SparseCompressKind kind);

  idx* OuterIndex() { return outer_index_.data(); }
  idx* InnerIndex() { return inner_index_.data(); }
  real* Values() { return values_.data(); }

  idx const* OuterIndex() const { return outer_index_.data(); }
  idx const* InnerIndex() const { return inner_index_.data(); }
  real const* Values() const { return values_.data(); }

  /************************* SECT: API *************************/
  virtual SparseCompressKind GetKind() const = 0;
  virtual void Assign(spmatr const& A) = 0;
  virtual void AssignSparsePatternUnchanged(spmatr const& A) = 0;
  virtual void AssignSparsePattern(spmatr const& A) = 0;

  // NOTE: These apis are important, but not necessary for now.
  // virtual spmatr ToStandard() const = 0;
  // virtual vecxr RightMultiply(vecxr const& x) const = 0;
  // virtual vecxr LeftMultiply(vecxr const& x) const = 0;

protected:
  std::vector<idx> outer_index_;
  std::vector<idx> inner_index_;
  std::vector<real> values_;
  idx rows_{0}, cols_{0};
};

}  // namespace ax::math
