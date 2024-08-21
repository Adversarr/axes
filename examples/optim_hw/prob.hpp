#pragma once
#include <cstdio>
#include <fstream>
#include <vector>

#include "ax/core/config.hpp"
#include "ax/core/logging.hpp"
#include "ax/core/excepts.hpp"
#include "ax/math/common.hpp"
#include "ax/math/linalg.hpp"
#include "ax/math/sparse.hpp"

namespace xx {

// Problem definition
// Logistic Reguression.
using namespace ax;
using namespace math;

constexpr Index PREDEFINED_M = 32561;
constexpr Index PREDEFINED_FEAT = 123;

class SPLR {
public:
  Real Energy(RealVectorX const& x) const {
    // x is m by 1
    return Energy_Loss(x) + Energy_L2(x) + Energy_L1(x);
  }

  Real Energy_L2(RealVectorX const& x) const {
    // x is m by 1
    real l2 = norm2(x);
    return lambda_ * l2;
  }
  Real Energy_Loss(RealVectorX const& x) const {
    // logistic regression loss.
    // x is FEAT by 1
    // bA_ is FEAT by M
    // bA_ * x is FEAT by 1

    // math::RealVectorX y = bA_.transpose() * x;
    math::RealVectorX y = A_.transpose() * x;
    y = y.cwiseProduct(b_);
    real loss = 0;
    for (Index i = 0; i < y.size(); ++i) {
      loss += std::log(1 + std::exp(y(i)));
    }
    return loss / PREDEFINED_M;
  }

  Real Energy_L1(RealVectorX const& x) const { return mu_ * x.cwiseAbs().sum(); }

  RealVectorX Gradient_Loss(RealVectorX const& x) const {
    // x is FEAT by 1
    // bA_ is FEAT by M
    // bA_ * x is FEAT by 1
    math::RealVectorX y = A_.transpose() * x;
    y = y.cwiseProduct(b_);
    math::RealVectorX grad = A_ * (1 - 1 / (1 + y.array().exp())).matrix().cwiseProduct(b_);
    return grad / PREDEFINED_M;
  }

  RealVectorX Gradient_Loss_L2(RealVectorX const& x) const { return 2 * lambda_ * x; }

  SPLR(RealSparseMatrix const& A, RealVectorX const& b) : A_(A), b_(b) {}

  real lambda_, mu_;
  const RealSparseMatrix A_;  // row = dim, col = m
  const RealVectorX b_;
};

inline SPLR load_from_file(std::string const& filename) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    throw std::runtime_error("Cannot open file " + filename);
  }

  Index m = 0, n = PREDEFINED_FEAT;
  math::RealVectorX b = math::RealVectorX::Zero(PREDEFINED_M);
  sp_coeff_list coeff_list;
  for (std::string line; std::getline(file, line);) {
    if (line.empty() || line[0] == '#') {
      continue;
    }

    // +1 i1:v1 i2:v2 ...
    std::istringstream iss(line);
    real y;
    iss >> y;
    b(m) = y;
    AX_THROW_IF_FALSE(y == 1 || y == -1, "Invalid label: " + std::to_string(y));
    for (std::string pair; iss >> pair;) {
      std::istringstream iss_pair(pair);
      Index i;
      real v;
      iss_pair >> i;
      iss_pair.ignore();
      iss_pair >> v;
      coeff_list.emplace_back(i - 1, m, v);
      AX_THROW_IF_TRUE(v != 1, "Invalid value: " + std::to_string(v));
    }
    ++m;
  }

  if (m != PREDEFINED_M) {
    throw std::runtime_error("Invalid number of samples: " + std::to_string(m));
  }
  std::cout << "Loaded " << m << " samples, " << n << " features." << std::endl;
  RealSparseMatrix A(PREDEFINED_FEAT, PREDEFINED_M);
  A.setFromTriplets(coeff_list.begin(), coeff_list.end());
  A.makeCompressed();
  return SPLR(A, -b);
}

}  // namespace xx
