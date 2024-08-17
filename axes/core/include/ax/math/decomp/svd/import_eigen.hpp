#pragma once
#include "ax/core/excepts.hpp"
#include "common.hpp"

namespace ax::math::decomp {

template <Index dim, typename Scalar> class JacobiSvd {
public:
  using mat_t = Matrix<Scalar, dim, dim>;
  using vec_t = Vector<Scalar, dim>;
  using result_t = SvdResult<dim, Scalar>;

  JacobiSvd() = default;

  AX_FORCE_INLINE result_t Solve(mat_t const& A) const {
    auto svd = A.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    // WARN: We are not checking the convergence of JacobiSVD.
    return SvdResult<dim, Scalar>{svd.matrixU(), svd.singularValues(), svd.matrixV()};
  }
};

template <Index dim, typename Scalar> class BDCSvd {
public:
  using mat_t = Matrix<Scalar, dim, dim>;
  using vec_t = Vector<Scalar, dim>;
  using result_t = SvdResult<dim, Scalar>;

  BDCSvd() = default;

  AX_FORCE_INLINE result_t Solve(mat_t const& A) const {
    Eigen::BDCSVD<mat_t> svd(A);
    if (svd.info() != Eigen::Success) {
      throw make_invalid_argument("Eigen::BDCSVD failed.");
    }
    return SvdResult<dim, Scalar>{svd.matrixU(), svd.singularValues(), svd.matrixV()};
  }
};

}  // namespace ax::math
