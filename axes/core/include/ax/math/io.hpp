#pragma once
#include "ax/math/common.hpp"
#include "ax/core/status.hpp"

namespace ax::math {

Status write_npy_v10(std::ostream& out, const real* p, size_t write_length, size_t f, size_t i,
                     size_t j);

// Writes a vector to a file in the NPY format.
Status write_npy_v10(std::string path, const vec<real, Eigen::Dynamic>& vec);

// Writes a matrix to a file in the NPY format.
Status write_npy_v10(std::string path, const mat<real, dynamic, dynamic>& mat);

// TODO: Load NPY file.
StatusOr<math::matxxr> read_npy_v10_real(std::string path);

StatusOr<math::matxxi> read_npy_v10_idx(std::string path);

}  // namespace ax::math