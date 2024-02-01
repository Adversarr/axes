#pragma once
#include "axes/utils/iota.hpp"
#include "common.hpp"

namespace ax::math {

/****************************** Definitions ******************************/

template <idx degree> class PolynomialN {
public:
  using coeff_container = vecr<degree + 1>;

  explicit PolynomialN(coeff_container const& coeff) : coeff_(coeff) {}

  AXES_FORCE_INLINE coeff_container const& Coeff() const { return coeff_; }

  AXES_FORCE_INLINE coeff_container& Coeff() { return coeff_; }

  AXES_FORCE_INLINE PolynomialN<degree - 1> Derivative() const {
    coeff_container coeff;
    for (idx i : utils::iota(degree)) {
      coeff[i] = coeff_[i + 1] * (i + 1);
    }
    return PolynomialN<degree - 1>(coeff);
  }

  AXES_FORCE_INLINE PolynomialN<degree + 1> Integral(real value_at_zero = 0) const {
    coeff_container coeff;
    coeff[0] = value_at_zero;
    for (idx i : utils::iota(degree + 2)) {
      coeff[i] = coeff_[i - 1] / cast<real>(i);
    }
    return PolynomialN<degree + 1>(coeff);
  }

  AXES_FORCE_INLINE real operator()(real x) const {
    real result = 0;
    real x_pow = 1;
    for (idx i : utils::iota(degree + 1)) {
      result += coeff_[i] * x_pow;
      x_pow *= x;
    }
    return result;
  }

private:
  coeff_container coeff_;
};

using ConstantPolynomial = PolynomialN<0>;
using LinearPolynomial = PolynomialN<1>;
using QuadraticPolynomial = PolynomialN<2>;
using CubicPolynomial = PolynomialN<3>;

/****************************** Operations ******************************/
template <idx deg1, idx deg2>
AXES_FORCE_INLINE PolynomialN<deg1 + deg2> operator*(PolynomialN<deg1> const& lhs,
                                                     PolynomialN<deg2> const& rhs) {
  typename PolynomialN<deg1 + deg2>::coeff_container coeff;
  for (idx i : utils::iota(deg1 + deg2 + 1)) {
    coeff[i] = 0;
    for (idx j : utils::iota(i + 1)) {
      coeff[i] += lhs.Coeff()[j] * rhs.Coeff()[i - j];
    }
  }
  return PolynomialN<deg1 + deg2>(coeff);
}

template <idx deg1, idx deg2>
AXES_FORCE_INLINE PolynomialN<deg1 + deg2> operator+(PolynomialN<deg1> const& lhs,
                                                     PolynomialN<deg2> const& rhs) {
  const idx out_deg = std::max<idx>(deg1, deg2);
  typename PolynomialN<out_deg>::coeff_container coeff;
  for (idx i : utils::iota(out_deg + 1)) {
    coeff[i] = 0;
    if (i <= deg1) {
      coeff[i] += lhs.Coeff()[i];
    }
    if (i <= deg2) {
      coeff[i] += rhs.Coeff()[i];
    }
  }
  return PolynomialN<out_deg>(coeff);
}

/****************************** Output Stream ******************************/
template <idx degree> std::ostream& operator<<(std::ostream& os, PolynomialN<degree> const& poly) {
  os << "PolynomialN<" << degree << ">[";
  for (idx i : utils::iota(degree + 1)) {
    os << poly.Coeff()[i];
    if (i != degree) {
      os << ", ";
    }
  }
  os << "]";
  return os;
}

}  // namespace ax::math
