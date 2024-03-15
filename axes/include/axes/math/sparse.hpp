#pragma once

#include <Eigen/SparseCore>

#include "axes/core/common.hpp"

namespace ax::math {

using sp_matxxr = Eigen::SparseMatrix<real, Eigen::ColMajor, idx>;

using sp_coeff = Eigen::Triplet<real, idx>;

using sp_coeff_list = std::vector<sp_coeff>;

AX_FORCE_INLINE sp_matxxr make_sparse_matrix(idx rows, idx cols,
                                             sp_coeff_list const& coeff_list) {
  sp_matxxr mat(rows, cols);
  mat.setFromTriplets(coeff_list.begin(), coeff_list.end());
  return mat;
}

}  // namespace ax::math
