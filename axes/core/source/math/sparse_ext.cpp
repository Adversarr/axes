#include "ax/math/sparse_ext.hpp"
#include "ax/math/sparse_compression/csc.hpp"

namespace ax::math {

std::unique_ptr<SparseCompressMatrixBase> SparseCompressMatrixBase::Create(SparseCompressKind kind) {
  switch (kind) {
    case SparseCompressKind::kCsc:
      return std::make_unique<SparseCompressMatrix_CSC>();
    // TODO: not implemented
    case SparseCompressKind::kCsr:
    //   return std::make_unique<SparseCompressMatrix_CSC>();
    // case SparseCompressKind::kCoo:
    //   return std::make_unique<SparseCompressMatrix_CSC>();
    default:
      return nullptr;
  }
}

}
