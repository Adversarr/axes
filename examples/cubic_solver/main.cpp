#include "ax/math/polynomial.hpp"
#include <cassert>
using namespace ax;
template <int dim>
void print_info(ax::math::RootInfo<dim> const& info) {
  printf("degree: %ld\n", info.degree_);
  for (Index i = 0; i < dim; ++i) {
    if (info.valid_[i]) {
      printf("Root %ld: %f\n", i, info.root_[i]);
    } else {
      printf("Root %ld: invalid\n", i);
    }
  }
}

void test_solve_linear() {
  ax::math::RootInfo<1> info = ax::math::solve_linear(2.0, -4.0, -10.0, 10.0);
  assert(info.valid_[0]);
  assert(info.root_[0] == 2.0);
}

void test_solve_quadratic() {
  ax::math::RootInfo<2> info = ax::math::solve_quadratic(1.0, -3.0, 2.0, -10.0, 10.0);
  assert(info.valid_[0]);
  assert(info.root_[0] == 1.0);
  assert(info.valid_[1]);
  assert(info.root_[1] == 2.0);
}

Index cnt_error = 0;

void test_solve_cubic(Real a = 1, Real b = 2, Real c = 3) {
  // (x - a) (x - b) (x - c) = x^3 - (a + b + c) x^2 + (ab + bc + ca) x - abc
  Real u = a + b + c, v = a * b + b * c + c * a, w = a * b * c;
  ax::math::RootInfo<3> info = ax::math::solve_cubic(1.0, -u, v, -w, 0, 1.0, 1e-10, 24);
  bool has_error = false;
  // find a.
  bool has_a = false, has_b = false, has_c = false;
  for (Index i = 0; i < 3; ++i) {
    if (info.valid_[i]) {
      if (math::approx(info.root_[i], 1e-7) == a) {
        has_a = true;
      }
      if (math::approx(info.root_[i], 1e-7) == b) {
        has_b = true;
      }
      if (math::approx(info.root_[i], 1e-7) == c) {
        has_c = true;
      }
    }
  }
  has_error = !has_a || !has_b || !has_c;
  if (has_error) {
    printf("%lf %lf %lf: ", a, b, c);
    print_info(info);
    cnt_error += 1;
  }
}

int main() {
  test_solve_linear();
  test_solve_quadratic();
  Real step = 0.0712;

  for (Real a = 0; a <= 1; a += step) {
    for (Real b = a; b <= 1; b += step) {
      for (Real c = b; c <= 1; c += step) {
        test_solve_cubic(a, b, c);
      }
    }
  }

  printf("Total error: %ld\n", cnt_error);
  return 0;
}