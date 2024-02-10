#pragma once

#include <Eigen/SparseCore>

#include "axes/core/common.hpp"

namespace ax::math {

using sp_matxxr = Eigen::SparseMatrix<real, Eigen::ColMajor, idx>;

using sp_coeff = Eigen::Triplet<real, idx>;

using sp_coeff_list = std::vector<sp_coeff>;

}  // namespace ax::math
