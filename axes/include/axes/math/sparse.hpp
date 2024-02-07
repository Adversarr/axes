#pragma once

#include <Eigen/SparseCore>

#include "axes/core/common.hpp"

namespace ax::math {

using sp_matxxr = Eigen::SparseMatrix<real, Eigen::ColMajor, idx>;

using SparseCoeff = Eigen::Triplet<real, idx>;

using SparseCoeffVec = std::vector<SparseCoeff>;

}  // namespace ax::math
