#include <absl/flags/flag.h>
#include <absl/log/globals.h>

#include <ax/math/common.hpp>
#include <cmath>

#include "ax/core/logging.hpp"
#include "ax/core/entt.hpp"
#include "ax/core/init.hpp"
#include "ax/math/functional.hpp"
#include "ax/math/io.hpp"
#include "ax/math/linalg.hpp"

using namespace ax;

// solves burgers' eqn:
//    u_t + (1/2 u^2)_x = 0
// with
//    u(x, t = 0) = sin(x) + 1/2
math::RealField1 current;
Index Nx;
Real dx;
Real M = 1;
Real lf_alpha = 2;
bool is_weno = true;
bool time_high_accuracy = false;

#define LF_FLUX 0
#define GODUNOV_FLUX 1

#define LIMITER_NONE 0
#define LIMITER_TVD 1
#define LIMITER_TVB 2

int flux_type = GODUNOV_FLUX;
int limiter_type = LIMITER_NONE;

// Godunov's numerical flux:
// $ hat(f)(u_l, u_r) = cases(
//   min_(u_l <= u <= u_r) f(u) quad & u_l <= u_r,
//   max_(u_r <= u <= u_l) f(u) quad & u_r <= u_l
// ) $
Real godunuv_flux(Real u_l, Real u_r) {
  if (u_l > u_r) {
    // Use max f(u)
    Real fl = 0.5 * u_l * u_l;
    Real fr = 0.5 * u_r * u_r;
    return std::max(fl, fr);
  } else /* u_l < u_r */ {
    // Use min f(u)
    if (u_l > 0) {
      return u_l * u_l / 2;
    } else if (u_r < 0) {
      return u_r * u_r / 2;
    } else {
      return 0;
    }
  }
}

void determine_lf_alpha() {
  lf_alpha = 1.5;
  for (Index i = 0; i < Nx; ++i) {
    Real u_l = current(i);
    lf_alpha = std::max(lf_alpha, std::abs(u_l));
  }
}

Real f(Real u) { return 0.5 * u * u; }

Real lf_flux(Real u_l, Real u_r) {
  // Lax-Fredrichs flux
  // 1/2 [f(u_l) + f(u_r)] - 1/2 alpha (u_r - u_l)
  // alpha: max |f' (u)| = max |u|
  // f(u) = 1/2 u^2
  // Real lf_alpha = std::max(math::abs(u_l), math::abs(u_r));
  return 0.5 * (f(u_l) + f(u_r)) - 0.5 * lf_alpha * (u_r - u_l);
}

Real initial_data(Real x, Real dx) {
  // Integrate[sin x + 1/2 x, x -> x + dx] = cos(x) - cos(x + dx) + 1/4 (x + dx)^2 - 1/4 x^2
  return (std::cos(x - 0.5 * dx) - std::cos(x + 0.5 * dx)) / dx + 0.5;
}

Real cfl() {
  // Determine delta t:
  // alpha Delta t <= Delta x / 2
  // alpha = max |f'| = max |u| = 1.5
  if (time_high_accuracy) {
    // To achieve 5 order,
    return 0.25 * dx / 1.5 * math::pow(dx, 5. / 3.);
  } else {
    return 0.25 * dx / 1.5;
  }
  // return 0.5 * dx;
}

Real minmod(Real a1, Real a2, Real a3) {
  if (a1 > 0 && a2 > 0 && a3 > 0) {
    return std::min({a1, a2, a3});
  } else if (a1 < 0 && a2 < 0 && a3 < 0) {
    return std::max({a1, a2, a3});
  } else {
    return 0;
  }
}

Real minmod_raise_to_a1(Real a1, Real a2, Real a3, Real M) {
  Real lim = M * dx * dx;
  if (std::abs(a1) > lim) {
    return minmod(a1, a2, a3);
  } else {
    return a1;
  }
}

Real tvd_limiter_right(Real u_l, Real u_c, Real u_r, Real u_reconstruct) {
  Real rc = u_r - u_c;
  Real cl = u_c - u_l;
  Real rec_c = u_reconstruct - u_c;
  return minmod(rec_c, rc, cl) + u_c;
}

Real tvd_limiter_left(Real u_l, Real u_c, Real u_r, Real u_reconstruct) {
  Real rc = u_r - u_c;
  Real cl = u_c - u_l;
  Real rec_c = u_c - u_reconstruct;
  return u_c - minmod(rec_c, rc, cl);
}

Real tvb_limiter_right(Real u_l, Real u_c, Real u_r, Real u_reconstruct) {
  Real rc = u_r - u_c;
  Real cl = u_c - u_l;
  Real rec_c = u_reconstruct - u_c;
  return minmod_raise_to_a1(rec_c, rc, cl, M) + u_c;
}

Real tvb_limiter_left(Real u_l, Real u_c, Real u_r, Real u_reconstruct) {
  Real rc = u_r - u_c;
  Real cl = u_c - u_l;
  Real rec_c = u_c - u_reconstruct;
  return u_c - minmod_raise_to_a1(rec_c, rc, cl, M);
}

Real no_limiter(Real /* u_l */, Real /* u_c */, Real /* u_r */, Real u_reconstruct) {
  return u_reconstruct;
}

math::RealField2 reconstruct_3rd_order_direct(math::RealField1 const& in) {
  math::RealField2 out(2, in.cols());
  // out(0, i) = u_{i+1/2}^-
  // out(1, i) = u_{i+1/2}^+
  for (Index i = 0; i < Nx; ++i) {
    Real u_l = in((i + Nx - 1) % Nx);
    Real u_c = in(i);
    Real u_r = in((i + 1) % Nx);
    Real u_rr = in((i + 2) % Nx);
    out(0, i) = -1.0 / 6.0 * u_l + 5.0 / 6.0 * u_c + 1.0 / 3.0 * u_r;
    out(1, i) = 1.0 / 3.0 * u_c + 5.0 / 6.0 * u_r - 1.0 / 6.0 * u_rr;
  }
  return out;
}

math::RealField3 weno_smooth_indicator(math::RealField1 const& u) {
  math::RealField3 beta(3, u.cols());
  for (Index i = 0; i < u.cols(); ++i) {
    Real u_ll = u((i + Nx - 2) % Nx);
    Real u_l = u((i + Nx - 1) % Nx);
    Real u_c = u(i);
    Real u_r = u((i + 1) % Nx);
    Real u_rr = u((i + 2) % Nx);

    Real d0 = 13.0 / 12.0 * (u_ll - 2.0 * u_l + u_c) * (u_ll - 2.0 * u_l + u_c)
              + 0.25 * (u_ll - 4.0 * u_l + 3.0 * u_c) * (u_ll - 4.0 * u_l + 3.0 * u_c);
    Real d1 = 13.0 / 12.0 * (u_l - 2.0 * u_c + u_r) * (u_l - 2.0 * u_c + u_r)
              + 0.25 * (u_l - u_r) * (u_l - u_r);
    Real d2 = 13.0 / 12.0 * (u_c - 2.0 * u_r + u_rr) * (u_c - 2.0 * u_r + u_rr)
              + 0.25 * (3.0 * u_c - 4.0 * u_r + u_rr) * (3.0 * u_c - 4.0 * u_r + u_rr);
    Real eps = 1e-12;
    Real alpha0 = 0.1 / ((d0 + eps) * (d0 + eps));
    Real alpha1 = 0.6 / ((d1 + eps) * (d1 + eps));
    Real alpha2 = 0.3 / ((d2 + eps) * (d2 + eps));
    Real sum_alpha = alpha0 + alpha1 + alpha2;
    beta(0, i) = alpha0 / sum_alpha;
    beta(1, i) = alpha1 / sum_alpha;
    beta(2, i) = alpha2 / sum_alpha;
  }
  return beta;
}

math::RealField3 reconstruct_3rd_order_weno_left(math::RealField1 const& in) {
  math::RealField3 left(3, in.cols());
  for (Index i = 0; i < Nx; ++i) {
    Real u_ll = in((i + Nx - 2) % Nx);
    Real u_l = in((i + Nx - 1) % Nx);
    Real u_c = in(i);
    Real u_r = in((i + 1) % Nx);
    Real u_rr = in((i + 2) % Nx);
    Real u_rrr = in((i + 3) % Nx);
    left(0, i) = (-u_ll + 5 * u_l + 2 * u_c) / 6;
    left(1, i) = (2 * u_l + 5 * u_c - u_r) / 6;
    left(2, i) = (11 * u_c - 7 * u_r + 2 * u_rr) / 6;
  }
  return left;
}

math::RealField3 reconstruct_3rd_order_weno_right(math::RealField1 const& in) {
  math::RealField3 right(3, in.cols());
  for (Index i = 0; i < Nx; ++i) {
    Real u_ll = in((i + Nx - 2) % Nx);
    Real u_l = in((i + Nx - 1) % Nx);
    Real u_c = in(i);
    Real u_r = in((i + 1) % Nx);
    Real u_rr = in((i + 2) % Nx);
    Real u_rrr = in((i + 3) % Nx);

    right(0, i) = (2 * u_ll - 7 * u_l + 11 * u_c) / 6;
    right(1, i) = (-u_l + 5 * u_c + 2 * u_r) / 6;
    right(2, i) = (2 * u_c + 5 * u_r - u_rr) / 6;
  }
  return right;
}

math::RealField2 reconstruct_3rd_order_weno(math::RealField1 const& in) {
  math::RealField3 left = reconstruct_3rd_order_weno_left(in);
  math::RealField3 right = reconstruct_3rd_order_weno_right(in);
  math::RealField3 beta = weno_smooth_indicator(in);
  math::RealField2 out(2, in.cols());
  for (Index i = 0; i < Nx; ++i) {
    out(0, i) = math::dot(right.col(i), beta.col(i));
    out(1, i) = math::dot(left.col((i + 1) % Nx), beta.col((i + 1) % Nx));
  }
  return out;
}

math::RealField1 rk1(math::RealField1 const& in, Real dt) {
  math::RealField1 out = in;
  Real dtdx = dt / dx;
  math::RealField2 u_c;
  if (is_weno) {
    u_c = reconstruct_3rd_order_weno(in);
  } else {
    u_c = reconstruct_3rd_order_direct(in);
    if (limiter_type != LIMITER_NONE) {
      math::RealField2 u_c_mod = u_c;
      for (Index i = 0; i < Nx; ++i) {
        Real ui_rec_left = u_c(1, (i - 1 + Nx) % Nx);
        Real ui_rec_right = u_c(0, i);
        if (limiter_type == LIMITER_TVD) {
          u_c_mod(1, (i - 1 + Nx) % Nx)
              = tvd_limiter_left(in((i - 1 + Nx) % Nx), in(i), in((i + 1) % Nx), ui_rec_left);
          u_c_mod(0, i)
              = tvd_limiter_right(in((i - 1 + Nx) % Nx), in(i), in((i + 1) % Nx), ui_rec_right);
        } else if (limiter_type == LIMITER_TVB) {
          u_c_mod(1, (i - 1 + Nx) % Nx)
              = tvb_limiter_left(in((i - 1 + Nx) % Nx), in(i), in((i + 1) % Nx), ui_rec_left);
          u_c_mod(0, i)
              = tvb_limiter_right(in((i - 1 + Nx) % Nx), in(i), in((i + 1) % Nx), ui_rec_right);
        }
      }
      u_c = u_c_mod;
    }
  }

  for (Index i = 0; i < Nx; ++i) {
    Real ui_minus = u_c(0, i);
    Real ui_plus = u_c(1, i);
    Real ui_1_minus = u_c(0, (i + Nx - 1) % Nx);
    Real ui_1_plus = u_c(1, (i + Nx - 1) % Nx);

    if (flux_type == LF_FLUX) {
      out(i) = in(i)-dtdx * (lf_flux(ui_minus, ui_plus) - lf_flux(ui_1_minus, ui_1_plus));
    } else {
      out(i) = in(i)-dtdx * (godunuv_flux(ui_minus, ui_plus) - godunuv_flux(ui_1_minus, ui_1_plus));
    }
  }
  return out;
}

math::RealField1 rk3(Real dt) {
  if (flux_type == LF_FLUX) {
    determine_lf_alpha();
  }
  math::RealField1 u1 = rk1(current, dt);
  math::RealField1 u2 = 0.75 * current + 0.25 * rk1(u1, dt);
  math::RealField1 u = (1.0 / 3.0) * current + (2.0 / 3.0) * rk1(u2, dt);
  return u;
}

ABSL_FLAG(Index, Nx, 100, "Number of grid points");
ABSL_FLAG(Index, Nt, 100, "Number of time steps");
ABSL_FLAG(Real, T, 1, "Final time");
ABSL_FLAG(int, flux_type, GODUNOV_FLUX, "Flux type");
ABSL_FLAG(int, limiter_type, LIMITER_NONE, "Limiter type");
ABSL_FLAG(bool, weno, true, "enable weno");
ABSL_FLAG(Real, M, 1, "M for TVB limiter");
ABSL_FLAG(bool, time_high_accuracy, false, "Time high accuracy");
ABSL_FLAG(std::string, input, "", "Input file");

int main(int argc, char** argv) {
  ax::init(argc, argv);
  Nx = absl::GetFlag(FLAGS_Nx);
  dx = 2 * math::pi<Real> / Nx;
  Real T = absl::GetFlag(FLAGS_T);
  flux_type = absl::GetFlag(FLAGS_flux_type);
  is_weno = absl::GetFlag(FLAGS_weno);
  if (flux_type == GODUNOV_FLUX) {
    AX_LOG(INFO) << "Using Godunov's flux";
  } else {
    AX_LOG(INFO) << "Using Lax-Fredrichs flux";
  }

  limiter_type = absl::GetFlag(FLAGS_limiter_type);
  time_high_accuracy = absl::GetFlag(FLAGS_time_high_accuracy);
  M = absl::GetFlag(FLAGS_M);
  if (limiter_type == LIMITER_NONE) {
    AX_LOG(INFO) << "No limiter";
  } else if (limiter_type == LIMITER_TVD) {
    AX_LOG(INFO) << "Using TVD limiter";
  } else {
    AX_LOG(INFO) << "Using TVB limiter";
  }

  AX_LOG(INFO) << "Delta x: " << dx;
  AX_LOG(INFO) << "CFL: " << cfl();

  // Do the algorithm.
  std::vector<math::RealVectorX> final_result_list;
  /* 1. Setup the initial data */
  current = math::RealField1(Nx);
  for (Index i = 0; i < Nx; ++i) {
    current(i) = initial_data(i * dx, dx);
  }
  if (std::string file = absl::GetFlag(FLAGS_input); !file.empty()) {
    current = math::read_npy_v10_real(file);
  }
  final_result_list.push_back(current.transpose());
  Real tt = 0;

  std::cout << "Expected Time step count = " << T / cfl() << std::endl;

  /* 2. Loop until finish */
  while (tt < T) {
    Real dt = std::min(cfl(), T - tt);
    current = rk3(dt);
    tt += cfl();
    final_result_list.push_back(current.transpose());
  }

  /* 3. Record all the solutions and export. */
  math::RealMatrixX uxt = math::RealMatrixX(Nx + 1, final_result_list.size());
  for (size_t i = 0; i < final_result_list.size(); ++i) {
    uxt.block(0, i, Nx, 1) = final_result_list[i];
  }
  uxt.row(Nx) = uxt.row(0);  // apply the periodic boundary condition

  std::string path = "u" + std::to_string(Nx) + "_" + std::to_string(Index(10 * T)) + ".npy";
  (math::write_npy_v10(path, uxt));

  ax::clean_up();
  return 0;
}
