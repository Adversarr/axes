#pragma once
#include "ax/math/approx.hpp"
#include "ax/math/functional.hpp"
#include "common.hpp"

namespace ax::math {

template <Index dim> struct RootInfo {
  bool valid_[dim];
  real root_[dim];
  Index degree_;
};

// a x + b = 0
RootInfo<1> AX_HOST_DEVICE AX_FORCE_INLINE solve_linear(real a, real b, real lb, real ub,
                                                        real tol = math::epsilon<real>) {
  RootInfo<1> info;
  info.valid_[0] = false;
  if (approx(a, tol) == 0.) {
    info.degree_ = 0;
    if (approx(b, tol) == 0.) {
      info.valid_[0] = true;
      info.root_[0] = 0.5 * (lb + ub);
    }
  } else {
    info.degree_ = 1;
    real root = -b / a;
    if (lb - tol < root && root < ub + tol) {
      info.valid_[0] = true;
      info.root_[0] = root;
    }
  }
  return info;
}

namespace details {

AX_HOST_DEVICE AX_FORCE_INLINE void step_newton_once(real& x, real a, real b, real c, real d,
                                                     RootInfo<2> const& info2,
                                                     real lb, real ub, real tol) {
  // f(x) = a x^3 + b x^2 + c x + d
  // f'(x) = 3 a x^2 + 2 b x + c
  // x = x - f(x) / f'(x)
  if (lb >= ub) return;
  real partial_f = 3 * a * x * x + 2 * b * x + c;
  real f = a * x * x * x + b * x * x + c * x + d;
  if (abs(f) < tol) {
    return;
  }
  x = clamp(x - f / partial_f, lb, ub);
}

}  // namespace details

// a x^2 + b x + c = 0
RootInfo<2> AX_HOST_DEVICE AX_FORCE_INLINE solve_quadratic(real a, real b, real c, real lb, real ub,
                                                           real tol = math::epsilon<real>) {
  RootInfo<2> info;
  info.valid_[0] = info.valid_[1] = false;
  if (approx(a, tol) == 0.) {
    auto i1 = solve_linear(b, c, lb, ub, tol);
    info.valid_[0] = i1.valid_[0];
    info.root_[0] = i1.root_[0];
    info.degree_ = i1.degree_;
    return info;
  }

  info.degree_ = 2;
  real const discriminant = b * b - 4 * a * c;
  if (discriminant >= -tol * tol) /* does have real root */ {
    if (discriminant <= tol * tol) {
      info.root_[0] = (info.root_[1] = -0.5 * b / a);
      info.valid_[0] = (info.valid_[1] = (lb - tol < info.root_[0] && info.root_[0] < ub + tol));
    } else {
      real const sqrt_discriminant = sqrt(discriminant);
      info.root_[0] = (-b - sqrt_discriminant) / (2 * a);
      info.valid_[0] = (lb - tol < info.root_[0] && info.root_[0] < ub + tol);
      Index const next = info.valid_[0] ? 1 : 0;
      info.root_[next] = (-b + sqrt_discriminant) / (2 * a);
      info.valid_[next] = (lb - tol < info.root_[next] && info.root_[next] < ub + tol);
    }
  }
  return info;
}

// a x^3 + b x^2 + c x + d = 0
RootInfo<3> AX_HOST_DEVICE AX_FORCE_INLINE solve_cubic(real a, real b, real c, real d, real lb,
                                                       real ub, real tol = math::epsilon<real>,
                                                       Index max_iteration = 16) {
  RootInfo<3> info;
  info.valid_[0] = info.valid_[1] = info.valid_[2] = false;
  if (approx(a, tol) == 0.) {
    auto i1 = solve_quadratic(b, c, d, lb, ub, tol);
    info.valid_[0] = i1.valid_[0];
    info.root_[0] = i1.root_[0];
    info.valid_[1] = i1.valid_[1];
    info.root_[1] = i1.root_[1];
    info.degree_ = i1.degree_;
  } else {
    info.degree_ = 3;
    // a != 0.
    // first find two stationary points, we assume our stationary point solver is stable enough.
    RootInfo<2> grad_info = solve_quadratic(3 * a, 2 * b, c, lb, ub, tol);
    real lower_bounds[3] = {ub, ub, ub};
    real upper_bounds[3] = {lb, lb, lb};
    if (grad_info.valid_[0] && grad_info.valid_[1]) /* both is valid */ {
      if (grad_info.root_[0] == grad_info.root_[1]) /* same root */ {
        lower_bounds[0] = grad_info.root_[0] - tol;
        upper_bounds[0] = ub + tol;
        info.root_[0] = 0.5 * (lower_bounds[0] + upper_bounds[0]);
        lower_bounds[1] = lb - tol;
        upper_bounds[1] = grad_info.root_[0] + tol;
        info.root_[1] = 0.5 * (lower_bounds[0] + upper_bounds[0]);
      } else {
        lower_bounds[0] = lb - tol;
        upper_bounds[0] = grad_info.root_[0] + tol;
        info.root_[0] = 0.5 * (lower_bounds[0] + upper_bounds[0]);
        lower_bounds[1] = grad_info.root_[0] - tol;
        upper_bounds[1] = grad_info.root_[1] + tol;
        info.root_[1] = 0.5 * (lower_bounds[1] + upper_bounds[1]);
        lower_bounds[2] = grad_info.root_[1] - tol;
        upper_bounds[2] = ub + tol;
        info.root_[2] = 0.5 * (lower_bounds[2] + upper_bounds[2]);
      }
      // do with the multiple roots.
      for (Index i = 0; i < 2; ++i) {
        real const x = grad_info.root_[i];
        real const f = a * x * x * x + b * x * x + c * x + d;
        if (abs(f) < tol) {
          info.valid_[i] = true;
          info.root_[i] = x;
        }
      }
    } else if (grad_info.valid_[0]) /* only first is valid */ {
      lower_bounds[0] = lb - tol;
      upper_bounds[0] = grad_info.root_[0] + tol;
      info.root_[0] = 0.5 * (lower_bounds[0] + upper_bounds[0]);
      lower_bounds[1] = grad_info.root_[0] - tol;
      upper_bounds[1] = ub + tol;
      info.root_[1] = 0.5 * (lower_bounds[1] + upper_bounds[1]);

      real const x = grad_info.root_[0];
      real const f = a * x * x * x + b * x * x + c * x + d;
      if (abs(f) < tol) {
        info.valid_[0] = true;
        info.root_[0] = x;
      }
    } else if (grad_info.valid_[1]) /* only second is valid */ {
      lower_bounds[0] = lb - tol;
      upper_bounds[0] = grad_info.root_[1] + tol;
      info.root_[0] = 0.5 * (lower_bounds[0] + upper_bounds[0]);
      lower_bounds[1] = grad_info.root_[1] - tol;
      upper_bounds[1] = ub + tol;
      info.root_[1] = 0.5 * (lower_bounds[1] + upper_bounds[1]);
      real const x = grad_info.root_[1];
      real const f = a * x * x * x + b * x * x + c * x + d;
      if (abs(f) < tol) {
        info.valid_[0] = true;
        info.root_[0] = x;
      }
    } else /* none is valid */ {
      lower_bounds[0] = lb - tol;
      upper_bounds[0] = ub + tol;
      info.root_[0] = 0.5 * (lower_bounds[0] + upper_bounds[0]);
    }

    // now we have 3 intervals to search for roots. run a newton's method in parallel.
    for (Index i = 0; i < 3; ++i) {
      if (lower_bounds[i] < upper_bounds[i]) {
        for (Index iter = 0; iter < max_iteration; ++iter) {
          details::step_newton_once(info.root_[i], a, b, c, d, grad_info, lower_bounds[i],
                                    upper_bounds[i], tol);
        }
      }
    }

    // check if the roots are valid.
    for (Index i = 0; i < 3; ++i) {
      real const l = lower_bounds[i], u = upper_bounds[i];
      if (l <= u) {
        real const x = info.root_[i];
        real const f = a * x * x * x + b * x * x + c * x + d;
        if (abs(f) < tol) {
          info.valid_[i] = true;
        }
      } else {
        info.valid_[i] = false;
      }
    }
    Index actual = 0;
    for (Index i = 0; i < 3; ++i) {
      if (info.valid_[i]) {
        info.root_[actual] = info.root_[i];
        info.valid_[actual] = true;
        ++actual;
      }
    }
    for (Index i = actual; i < 3; ++i) {
      info.valid_[i] = false;
    }
  }
  return info;
}

}  // namespace ax::math