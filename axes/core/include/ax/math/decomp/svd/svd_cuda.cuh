#pragma once
#include "ax/math/common.hpp"
#include "svd3_cuda.cuh"

namespace ax::math::decomp {

AX_DEVICE AX_FORCE_INLINE void svd(math::mat3f const& A, math::mat3f& U, math::mat3f& V,
                                   math::vec3f& Sigma) {
  ::svd(A(0, 0), A(0, 1), A(0, 2), A(1, 0), A(1, 1), A(1, 2), A(2, 0), A(2, 1), A(2, 2), U(0, 0),
        U(0, 1), U(0, 2), U(1, 0), U(1, 1), U(1, 2), U(2, 0), U(2, 1), U(2, 2), Sigma[0], Sigma[1],
        Sigma[2], V(0, 0), V(0, 1), V(0, 2), V(1, 0), V(1, 1), V(1, 2), V(2, 0), V(2, 1), V(2, 2));
}

AX_DEVICE AX_FORCE_INLINE void svd(math::mat2f const& A, math::mat2f& U, math::mat2f& V,
                                   math::vec2f& Sigma) {
  float a = A(0, 0), b = A(0, 1), c = A(1, 0), d = A(1, 1);
  float t1 = a * a + b * b - c * c - d * d;
  float t2 = a * c + b * d;
  float theta = 0.5 * atan2(2 * t2, t1);
  U(1, 1) = U(0, 0) = cos(theta);
  float sin_theta = sin(theta);
  U(0, 1) = -sin_theta;
  U(1, 0) = sin_theta;
  float s1 = a * a + b * b + c * c + d * d;
  float s2 = sqrt(t1 * t1 + 4 * t2 * t2);
  Sigma[0] = s1;
  Sigma[1] = s2;
  float sgn_s1 = s1 > 0 ? 1 : -1;
  float sgn_s2 = s2 > 0 ? 1 : -1;
  float phi = 0.5 * atan2(2 * (a * b + c * d), a * a - b * b + c * c - d * d);
  V(0, 0) = sgn_s1 * cos(phi);
  V(1, 0) = sgn_s1 * sin(phi);
  V(0, 1) = -sgn_s2 * sin(phi);
  V(1, 1) = sgn_s2 * cos(phi);
}

}  // namespace ax::math::decomp
