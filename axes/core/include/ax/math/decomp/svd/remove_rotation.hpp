#pragma once

#include "common.hpp"
namespace ax::math::decomp {

template<typename Scalar>
bool AX_FORCE_INLINE svd_remove_rotation(SvdResultImpl<3, Scalar> & svd) {
  bool has_rotation = false;
  Scalar det_u = svd.U_.determinant();
  Scalar det_v = svd.V_.determinant();
  Scalar L = (svd.U_ * svd.V_.transpose()).determinant();
  if (det_u < 0 && det_v > 0) {
    svd.U_.col(2) *= L;
    has_rotation = true;
  } else if (det_u > 0 && det_v < 0) {
    svd.V_.col(2) *= L;
    has_rotation = true;
  }
  svd.sigma_[2] *= L;
  return has_rotation;
}

template<typename Scalar>
bool AX_FORCE_INLINE svd_remove_rotation(SvdResultImpl<2, Scalar> & svd) {
  // XXX: This function is automatically generated by the tool?
  bool has_rotation = false;
  Scalar det_u = svd.U_.determinant();
  Scalar det_v = svd.V_.determinant();
  Scalar L = (svd.U_ * svd.V_.transpose()).determinant();
  if (det_u < 0 && det_v > 0) {
    svd.U_.col(1) *= L;
    has_rotation = true;
  } else if (det_u > 0 && det_v < 0) {
    svd.V_.col(1) *= L;
    has_rotation = true;
  }
  svd.sigma_[1] *= L;
  return has_rotation;
}
}