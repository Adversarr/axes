#pragma once
#include "axes/math/common.hpp"
#include "axes/core/status.hpp"

namespace ax::math {

Status write_npy_v10(std::ostream& out, const real* p, size_t write_length, size_t f, size_t i,
                     size_t j);

Status write_npy_v10(std::string path, const vec<real, Eigen::Dynamic>& vec);

Status write_npy_v10(std::string path, const mat<real, dynamic, dynamic>& mat);

}  // namespace ax::math