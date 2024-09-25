#pragma once
#include "ax/math/sparse_matrix/block_matrix.hpp"

namespace ax::math::details {

void block_matrix_to_csr(const math::RealBlockMatrix& block, math::RealCSRMatrix& csr);

}  // namespace ax::math::details