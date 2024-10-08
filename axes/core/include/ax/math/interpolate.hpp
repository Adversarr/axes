#pragma once
#include "ax/math/ndrange.hpp"
#include "ax/utils/god.hpp"

namespace ax::math {

template <typename T, typename Scalar = Real>
AX_HOST_DEVICE AX_FORCE_INLINE T lerp(T const& a, T const& b, Scalar const& t) {
  return a * (1.0 - t) + b * t;
}

template <typename T, typename Scalar = Real>
AX_HOST_DEVICE AX_FORCE_INLINE T bilerp(T const& a, T const& b, T const& c, T const& d,
                                             Scalar const& s, Scalar const& t) {
  return lerp(lerp(a, b, s), lerp(c, d, s), t);
}

template <typename T, typename Scalar = Real>
AX_HOST_DEVICE AX_FORCE_INLINE T bilerp(std::array<T, 4> points, Scalar const& s,
                                             Scalar const& t) {
  return bilerp(points[0], points[1], points[2], points[3], s, t);
}

template <typename T, typename Scalar = Real>
AX_HOST_DEVICE AX_FORCE_INLINE T trilerp(std::array<T, 8> points, Scalar const& s,
                                              Scalar const& t, Scalar const& u) {
  return lerp(bilerp(points[0], points[1], points[2], points[3], s, t),
              bilerp(points[4], points[5], points[6], points[7], s, t), u);
}

template <typename Scalar = Real>
AX_HOST_DEVICE AX_FORCE_INLINE Scalar linear_kernel(Scalar x) {
  Scalar abs_x = std::abs(x);
  return std::clamp(1.0 - abs_x, 0.0, 1.0);
}

template <typename Scalar = Real>
AX_HOST_DEVICE AX_FORCE_INLINE Scalar quadratic_kernel(Scalar x) {
  Scalar abs_x = std::abs(x);
  if (abs_x < 0.5) {
    return 0.75 - abs_x * abs_x;
  } else if (abs_x < 1.5) {
    return 0.5 * (1.5 - abs_x) * (1.5 - abs_x);
  } else {
    return 0.0;
  }
}

template <typename Scalar = Real>
AX_HOST_DEVICE AX_FORCE_INLINE Scalar cubic_kernel(Scalar x) {
  Scalar abs_x = std::abs(x);
  if (abs_x < 1.0) {
    return (1.5 * abs_x - 2.5) * abs_x * abs_x + 1.0;
  } else if (abs_x < 2.0) {
    return ((-0.5 * abs_x + 2.5) * abs_x - 4.0) * abs_x + 2.0;
  } else {
    return 0.0;
  }
}

template <int dim>
AX_HOST_DEVICE AX_FORCE_INLINE RealVector<utils::god::pow(3, dim)> quadratic_coefficient(
    RealVector<dim> const& delta) {
  RealVector<utils::god::pow(3, dim)> output;
  Index cnt = 0;
  for (auto ijk : math::ndrange(IndexVector<dim>::Constant(3))) {
    IndexVector<dim> sub = tuple_to_vector<Index, 3>(ijk);
    RealVector<dim> pos = sub.template cast<Real>() - math::ones<dim>() + delta;
    output[cnt++] = prod(pos.unaryExpr(&quadratic_kernel<Real>));
  }
  return output;
}

}  // namespace ax::math