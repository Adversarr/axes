#pragma once
#include <cstdio>
#include <fstream>
#include <vector>

#include "ax/core/config.hpp"
#include "ax/core/echo.hpp"
#include "ax/math/common.hpp"
#include "ax/math/linalg.hpp"
#include "ax/math/sparse.hpp"

namespace xx {

// Problem definition
// Logistic Reguression.
using namespace ax;
using namespace math;

constexpr idx PREDEFINED_M = 32561;
constexpr idx PREDEFINED_FEAT = 123;

class SPLR {
public:
  real Energy(vecxr const& x) const {
    // x is m by 1
    return Energy_Loss(x) + Energy_L2(x) + Energy_L1(x);
  }

  real Energy_L2(vecxr const& x) const {
    // x is m by 1
    real l2 = norm2(x);
    return lambda_ * l2;
  }
  real Energy_Loss(vecxr const& x) const {
    vecxr bAx = bA_.transpose() * x;
    real loss = (bAx.array().exp() + 1).log().mean();
    return loss;
  }

  real Energy_L1(vecxr const& x) const {
    real l1 = norm(x, l1_t{});
    return mu_ * l1;
  }

  vecxr Gradient(vecxr const& x) const {
    vecxr grad = Gradient_Loss(x);
    grad.noalias() += lambda_ * x;
    grad.noalias() += mu_ * x.cwiseSign();
    return grad;
  }
  vecxr Gradient_Loss(vecxr const& x) const {
    vecxr bAx = bA_.transpose() * x;
    vecxr grad = bA_ * (bAx.array().exp() / (bAx.array().exp() + 1)).matrix();
    return grad / PREDEFINED_M;
  }

  vecxr Gradient_Loss_L2(vecxr const& x) const { return lambda_ * x; }

  vecxr Gradient_Loss_L1(vecxr const& x) const { return mu_ * x.cwiseSign(); }

  SPLR(sp_matxxr const& A, vecxr const& b) {
    bA_ = A;
    for (idx i = 0; i < A.outerSize(); ++i) {
      for (sp_matxxr::InnerIterator it(A, i); it; ++it) {
        it.valueRef() = it.value() * b(it.col());
      }
    }
  }

  real lambda_, mu_;
  sp_matxxr bA_;  // row = dim, col = m
};

inline SPLR load_from_file(std::string const& filename) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    throw std::runtime_error("Cannot open file " + filename);
  }

  idx m = 0, n = PREDEFINED_FEAT;
  math::vecxr b = math::vecxr::Zero(PREDEFINED_M);
  sp_coeff_list coeff_list;
  for (std::string line; std::getline(file, line);) {
    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::vector<std::string> split_by_whitespace;
    idx j = 0;
    for (idx i = 0; i < line.size(); ++i) {
      if (line[i] == ' ' || line[i] == '\t' || i == line.size() - 1) {
        if (j < i) {
          split_by_whitespace.push_back(line.substr(j, i - j));
        }
        j = i + 1;
      }
    }
    if (split_by_whitespace.empty()) {
      throw std::runtime_error("Invalid line: " + line);
    }
    int bi;
    int p = sscanf(split_by_whitespace[0].c_str(), "%d", &bi);
    // +1 i1:v1 i2:v2 ...
    if (!(bi == 1 || bi == -1) || p != 1) {
      throw std::runtime_error("Invalid label: " + line);
    }
    for (size_t i = 1; i < split_by_whitespace.size(); ++i) {
      int row, aim;
      int ret = sscanf(split_by_whitespace[i].c_str(), "%d:%d", &row, &aim);
      if (row < 1 || aim == 0 || ret != 2) {
        throw std::runtime_error("Invalid feature: " + line);
      }
      coeff_list.push_back(sp_coeff(row - 1, m, aim));
    }
    b(m) = bi;
    ++m;
  }

  if (m != PREDEFINED_M) {
    throw std::runtime_error("Invalid number of samples: " + std::to_string(m));
  }
  std::cout << "Loaded " << m << " samples, " << n << " features." << std::endl;
  sp_matxxr A(PREDEFINED_FEAT, PREDEFINED_M);
  A.setFromTriplets(coeff_list.begin(), coeff_list.end());
  A.makeCompressed();
  return SPLR(A, -b);
}

}  // namespace xx