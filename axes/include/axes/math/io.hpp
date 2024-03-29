#pragma once
#include "axes/math/common.hpp"
#include "axes/core/status.hpp"

namespace ax::math {

Status write_npy_v10(std::ostream& out, const real* p, size_t write_length, size_t f, size_t i,
                     size_t j);

// Writes a vector to a file in the NPY format.
Status write_npy_v10(std::string path, const vec<real, Eigen::Dynamic>& vec);

// Writes a matrix to a file in the NPY format.
Status write_npy_v10(std::string path, const mat<real, dynamic, dynamic>& mat);

// TODO: Load NPY file.

}  // namespace ax::math