#pragma once
#include "ax/utils/status.hpp"
#include "common.hpp"

namespace ax::math::decomp {

template <idx dim, typename Scalar> class JacobiSvd {
public:
  using mat_t = mat<Scalar, dim, dim>;
  using vec_t = vec<Scalar, dim>;
  using result_t = SvdResult<dim, Scalar>;

  JacobiSvd() = default;

  AX_FORCE_INLINE result_t Solve(mat_t const& A) const {
    auto svd = A.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    if (svd.info() != Eigen::Success) {
      return utils::InvalidArgumentError("Eigen::JacobiSVD failed.");
    }
    return SvdResultImpl<dim, Scalar>{svd.matrixU(), svd.singularValues(), svd.matrixV()};
  }
};

template <idx dim, typename Scalar> class BDCSvd {
public:
  using mat_t = mat<Scalar, dim, dim>;
  using vec_t = vec<Scalar, dim>;
  using result_t = SvdResult<dim, Scalar>;

  BDCSvd() = default;

  AX_FORCE_INLINE result_t Solve(mat_t const& A) const {
    Eigen::BDCSVD<mat_t> svd(A);
    if (svd.info() != Eigen::Success) {
      return utils::InvalidArgumentError("Eigen::BDCSVD failed.");
    }
    return SvdResultImpl<dim, Scalar>{svd.matrixU(), svd.singularValues(), svd.matrixV()};
  }
};

}  // namespace ax::math
