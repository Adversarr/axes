#pragma once
#include "ax/utils/status.hpp"
#include "common.hpp"

namespace ax::math {

template <idx dim, typename Scalar> class JacobiSvd {
public:
  using mat_t = mat<Scalar, dim, dim>;
  using vec_t = vec<Scalar, dim>;
  using result_t = SvdResult<dim, Scalar>;

  JacobiSvd() = default;

  result_t Solve(mat_t const& A) const {
    Eigen::JacobiSVD<mat_t, Eigen::ComputeFullU | Eigen::ComputeFullV> svd(A);
    if (svd.info() != Eigen::Success) {
      return utils::InvalidArgumentError("Eigen::JacobiSVD failed.");
    }
    return {svd.matrixU(), svd.singularValues(), svd.matrixV()};
  }
};

template <idx dim, typename Scalar> class BDCSvd {
public:
  using mat_t = mat<Scalar, dim, dim>;
  using vec_t = vec<Scalar, dim>;
  using result_t = SvdResult<dim, Scalar>;

  BDCSvd() = default;

  result_t Solve(mat_t const& A) const {
    Eigen::BDCSVD<mat_t> svd(A);
    if (svd.info() != Eigen::Success) {
      return utils::InvalidArgumentError("Eigen::BDCSVD failed.");
    }
    return {svd.matrixU(), svd.singularValues(), svd.matrixV()};
  }
};

}  // namespace ax::math
