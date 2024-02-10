#pragma once
#include "polynomial.hpp"
namespace ax::math {

template<idx d, idx k>
class MultVariatePolynomial {
public:
  MultVariatePolynomial() = default;

  std::array<MultVariatePolynomial<d-1, k>, k + 1> sub_polys_;
};

}