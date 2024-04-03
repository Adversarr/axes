#pragma once

#include "ax/geometry/common.hpp"
#include "ax/math/linalg.hpp"

namespace ax::geo {

/****************************** Interface ******************************/

template <typename T> struct Measurement {
  real operator()(T const&) const {
    static_assert(sizeof(T) == -1, "Measurement not implemented for this type");
  }
};

template <typename T> real measure(T const& t) { return Measurement<T>{}(t); }

/****************************** Implementations ******************************/

template <> struct Measurement<SimplexN<2>> {
  real operator()(SimplexN<2> const& simplex) const {
    using namespace math;
    math::vec3r a;
    a << simplex[0], 0;
    math::vec3r b;
    b << simplex[1], 0;
    math::vec3r c;
    c << simplex[2], 0;
    math::vec3r n = math::cross(b - a, c - a);
    return 0.5 * math::norm(n);
  }
};

template <> struct Measurement<SimplexN<3>> {
  real operator()(SimplexN<3> const& simplex) const {
    using namespace math;
    mat3r m;
    m << simplex[1], simplex[2], simplex[3];
    each(m) -= simplex[0];
    return 1.0 / 6.0 * abs(det(m));
  }
};

}  // namespace ax::geo
