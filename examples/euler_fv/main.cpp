/**
 * Implements 1D Euler Equation Finite Volume Solver.(3rd order)
 */

#include "ax/core/echo.hpp"
#include "ax/core/init.hpp"
#include "ax/math/common.hpp"
#include "ax/math/io.hpp"
#include "ax/math/linalg.hpp"

using namespace ax;
using namespace ax::math;
using namespace std;
using scalar = ax::real;
using v3 = vec3r;

#define PROBLEM_SOD 0
#define FLUX_LAX_FRIEDRICHS 0
#define FLUX_HLL 1

/************************* SECT: Solution *************************/
scalar tend;
int nx;
bool periodic;
int problem;
int flux_type;
field1r density;
field1r momentum;
field1r energy;
scalar x_min, x_max;
string input;

// Ideal gas equation of state:
// E = p / (gamma - 1) + 0.5 * rho * u^2

constexpr scalar GAMMA = 1.4;
AX_FORCE_INLINE scalar pressure(scalar density, scalar momentum, scalar energy) {
  return (energy - 0.5 * momentum * momentum / density) * (GAMMA - 1);
}

AX_FORCE_INLINE scalar sound_speed(scalar density, scalar momentum, scalar energy) {
  return sqrt(GAMMA * pressure(density, momentum, energy) / density);
}
AX_FORCE_INLINE scalar compute_energy(scalar density, scalar momentum, scalar pressure) {
  return pressure / (GAMMA - 1) + 0.5 * momentum * momentum / density;
}

scalar cfl(scalar dx) {
  scalar max_speed = 1;
  for (idx i = 0; i < nx; ++i) {
    scalar speed = abs(momentum(i) / density(i));
    max_speed = max(max_speed, speed + sound_speed(density(i), momentum(i), energy(i)));
  }
  return dx / max_speed * 0.1;
}

AX_FORCE_INLINE v3 fu(scalar density, scalar momentum, scalar energy) {
  v3 f;
  scalar speed = momentum / density;
  scalar pres = pressure(density, momentum, energy);
  f.x() = momentum;
  f.y() = momentum * speed + pres;
  f.z() = speed * (energy + pres);
  return f;
}

AX_FORCE_INLINE v3 fu(const v3& u) { return fu(u.x(), u.y(), u.z()); }

AX_FORCE_INLINE v3 lax_friedrichs(const v3& left, const v3& right) {
  scalar const left_speed = left.y() / left.x();
  scalar const right_speed = right.y() / right.x();
  scalar const speed = 1.4 * max(abs(left_speed), abs(right_speed));
  v3 const f_left = fu(left);
  v3 const f_right = fu(right);
  return 0.5 * (f_left + f_right - speed * (right - left));
}

AX_FORCE_INLINE v3 hll(const v3& left, const v3& right) {
  scalar const left_speed = left.y() / left.x();
  scalar const right_speed = right.y() / right.x();
  scalar const left_energy = left.z();
  scalar const right_energy = right.z();
  scalar const left_pressure = pressure(left.x(), left.y(), left.z());
  scalar const right_pressure = pressure(right.x(), right.y(), right.z());
  scalar const left_sound_speed = sound_speed(left.x(), left.y(), left.z());
  scalar const right_sound_speed = sound_speed(right.x(), right.y(), right.z());
  scalar const p_star = (left_pressure * right_sound_speed + right_pressure * left_sound_speed
                         + left_speed * left.x() * (left_pressure - right_pressure))
                        / (left_sound_speed + right_sound_speed);
  scalar const u_star
      = (left_speed * left.x() + right_speed * right.x() + right_pressure - left_pressure)
        / (left.x() + right.x());
  scalar const s_l = min(left_speed - left_sound_speed, u_star - sqrt(GAMMA * p_star / left.x()));
  scalar const s_r
      = max(right_speed + right_sound_speed, u_star + sqrt(GAMMA * p_star / right.x()));
  if (0 <= s_l) {
    return fu(left);
  } else if (0 >= s_r) {
    return fu(right);
  } else {
    return (s_r * fu(left) - s_l * fu(right) + s_l * s_r * (right - left)) / (s_r - s_l);
  }
}

void initialize() {
  density.resize(nx);
  momentum.resize(nx);
  energy.resize(nx);

  if (!input.empty()) {
    // assuming that the input is [-pi, pi]
    x_min = -pi<>;
    x_max = pi<>;
    density = read_npy_v10_real(input + "_density.npy").transpose();
    momentum = read_npy_v10_real(input + "_momentum.npy").transpose();
    energy = read_npy_v10_real(input + "_energy.npy").transpose();
    return;
  }

  if (periodic) {
    x_min = -pi<>;
    x_max = pi<>;
    scalar dx = (x_max - x_min) / nx;
    for (idx i = 0; i < nx; ++i) {
      scalar x = x_min + i * dx;
      // 1+0.2 sin x, 1, 1
      density(i) = 1 + 0.2 * (cos(x) - cos(x + dx)) / dx;
      momentum(i) = density(i);
      energy(i) = 2.5 + 0.5 * density(i);
    }
  } else {
    x_min = -6;
    x_max = 6;
    if (problem == PROBLEM_SOD) {
      x_min = -1;
      x_max = 1;
    }

    scalar dx = (x_max - x_min) / nx;
    for (idx i = 0; i < nx; ++i) {
      scalar x = x_min + i * dx;
      if (x < 0) {
        density(i) = 1;
        momentum(i) = 0;
        energy(i) = compute_energy(density(i), momentum(i), 1.0);
      } else {
        density(i) = 0.125;
        momentum(i) = 0;
        energy(i) = 0.1 / (GAMMA - 1);
      }
    }
  }
}

AX_FORCE_INLINE scalar field_get(const field1r& f, idx i) {
  if (periodic) {
    return f((i + nx) % nx);
  } else {
    return f(std::clamp<idx>(i, 0, nx - 1));
  }
}

AX_FORCE_INLINE vec2r field_get(const field2r& f, idx i) {
  if (periodic) {
    return f.col((i + nx) % nx);
  } else {
    return f.col(std::clamp<idx>(i, 0, nx - 1));
  }
}

AX_FORCE_INLINE vec3r field_get(const field3r& f, idx i) {
  if (periodic) {
    return f.col((i + nx) % nx);
  } else {
    return f.col(std::clamp<idx>(i, 0, nx - 1));
  }
}

/************************* SECT: Algorithm *************************/

math::field3r weno_smooth_indicator(math::field1r const& u) {
  math::field3r beta(3, u.cols());
#pragma unroll 4
  for (idx i = 0; i < u.cols(); ++i) {
    scalar u_ll = field_get(u, (i - 2));
    scalar u_l = field_get(u, (i - 1));
    scalar u_c = u(i);
    scalar u_r = field_get(u, i + 1);
    scalar u_rr = field_get(u, i + 2);

    scalar d0 = 13.0 / 12.0 * (u_ll - 2.0 * u_l + u_c) * (u_ll - 2.0 * u_l + u_c)
                + 0.25 * (u_ll - 4.0 * u_l + 3.0 * u_c) * (u_ll - 4.0 * u_l + 3.0 * u_c);
    scalar d1 = 13.0 / 12.0 * (u_l - 2.0 * u_c + u_r) * (u_l - 2.0 * u_c + u_r)
                + 0.25 * (u_l - u_r) * (u_l - u_r);
    scalar d2 = 13.0 / 12.0 * (u_c - 2.0 * u_r + u_rr) * (u_c - 2.0 * u_r + u_rr)
                + 0.25 * (3.0 * u_c - 4.0 * u_r + u_rr) * (3.0 * u_c - 4.0 * u_r + u_rr);
    scalar eps = math::epsilon<>;
    scalar alpha0 = 0.1 / ((d0 + eps) * (d0 + eps));
    scalar alpha1 = 0.6 / ((d1 + eps) * (d1 + eps));
    scalar alpha2 = 0.3 / ((d2 + eps) * (d2 + eps));
    scalar sum_alpha = alpha0 + alpha1 + alpha2;
    beta(0, i) = alpha0 / sum_alpha;
    beta(1, i) = alpha1 / sum_alpha;
    beta(2, i) = alpha2 / sum_alpha;
  }
  return beta;
}

math::field3r reconstruct_3rd_order_weno_left(math::field1r const& in) {
  math::field3r left(3, in.cols());
#pragma unroll 4
  for (idx i = 0; i < nx; ++i) {
    scalar u_ll = field_get(in, (i - 2));
    scalar u_l = field_get(in, (i - 1));
    scalar u_c = in(i);
    scalar u_r = field_get(in, i + 1);
    scalar u_rr = field_get(in, i + 2);
    left(0, i) = (-u_ll + 5 * u_l + 2 * u_c) / 6;
    left(1, i) = (2 * u_l + 5 * u_c - u_r) / 6;
    left(2, i) = (11 * u_c - 7 * u_r + 2 * u_rr) / 6;
  }
  return left;
}

math::field3r reconstruct_3rd_order_weno_right(math::field1r const& in) {
  math::field3r right(3, in.cols());
#pragma unroll 4
  for (idx i = 0; i < nx; ++i) {
    scalar u_ll = field_get(in, (i - 2));
    scalar u_l = field_get(in, (i - 1));
    scalar u_c = in(i);
    scalar u_r = field_get(in, i + 1);
    scalar u_rr = field_get(in, i + 2);

    right(0, i) = (2 * u_ll - 7 * u_l + 11 * u_c) / 6;
    right(1, i) = (-u_l + 5 * u_c + 2 * u_r) / 6;
    right(2, i) = (2 * u_c + 5 * u_r - u_rr) / 6;
  }
  return right;
}

math::field2r reconstruct_3rd_order_weno(math::field1r const& in) {
  math::field3r left = reconstruct_3rd_order_weno_left(in);
  math::field3r right = reconstruct_3rd_order_weno_right(in);
  math::field3r beta = weno_smooth_indicator(in);
  math::field2r out(2, in.cols());
#pragma unroll 4
  for (idx i = 0; i < nx; ++i) {
    out(0, i) = math::dot(right.col(i), beta.col(i));
    v3 const l = field_get(left, i + 1), b = field_get(beta, i + 1);
    out(1, i) = math::dot(l, b);
  }
  return out;
}

AX_FORCE_INLINE void apply_flux(field1r& target, field1r const& f, scalar dtdx) {
#pragma unroll 4
  for (idx i = 0; i < nx; ++i) {
    target(i) -= dtdx * (field_get(f, i) - field_get(f, i - 1));
  }
}

void rk1(field1r& density, field1r& momentum, field1r& energy, scalar dtdx) {
  field2r const density_reconstructed = reconstruct_3rd_order_weno(density);
  field2r const momentum_reconstructed = reconstruct_3rd_order_weno(momentum);
  field2r const energy_reconstructed = reconstruct_3rd_order_weno(energy);
  field1r density_flux(1, nx), momentum_flux(1, nx), energy_flux(1, nx);

  for (idx i = 0; i < nx; ++i) {
    v3 const left(density_reconstructed(0, i), momentum_reconstructed(0, i),
                  energy_reconstructed(0, i));
    v3 const right(density_reconstructed(1, i), momentum_reconstructed(1, i),
                   energy_reconstructed(1, i));
    v3 flux;
    if (flux_type == FLUX_LAX_FRIEDRICHS) {
      flux = lax_friedrichs(left, right);
    } else if (flux_type == FLUX_HLL) {
      flux = hll(left, right);
    } else {
      throw std::runtime_error("Unknown flux type");
    }
    density_flux(0, i) = flux.x();
    momentum_flux(0, i) = flux.y();
    energy_flux(0, i) = flux.z();
  }

  apply_flux(density, density_flux, dtdx);
  apply_flux(momentum, momentum_flux, dtdx);
  apply_flux(energy, energy_flux, dtdx);
}

void copy_global(field1r& d, field1r& m, field1r& e) {
  d = density;
  m = momentum;
  e = energy;
}

void blend_to_global(field1r& d, field1r& m, field1r& e, scalar alpha) {
  density = alpha * d + (1 - alpha) * density;
  momentum = alpha * m + (1 - alpha) * momentum;
  energy = alpha * e + (1 - alpha) * energy;
}

void rk3(scalar dtdx) {
  static field1r density1(1, nx), momentum1(1, nx), energy1(1, nx);
  // 1 = rk1
  // 2 = 0.75 * prev + 0.25 * 1
  // 3 = 1/3 * prev + 2/3 * 2
  copy_global(density1, momentum1, energy1);
  rk1(density, momentum, energy, dtdx);  // now density is u1
  rk1(density, momentum, energy, dtdx);
  blend_to_global(density1, momentum1, energy1, 0.75);  // now density is u2
  rk1(density, momentum, energy, dtdx);
  blend_to_global(density1, momentum1, energy1, 1.0 / 3.0);  // now density is u3
  // rk1(density, momentum, energy, dtdx);
}

/************************* SECT: Flags *************************/
ABSL_FLAG(int, nx, 128, "Number of cells in x direction");
ABSL_FLAG(double, tend, 0.1, "End time of simulation");
ABSL_FLAG(int, problem, 0, "Problem number");
ABSL_FLAG(bool, periodic, true,
          "Periodic boundary condition. If periodic, the range is [-pi, pi], otherwise [-6, 6], "
          "notice your nx.");
ABSL_FLAG(int, flux_type, FLUX_HLL, "Flux type");
ABSL_FLAG(string, out, "output", "Output file");
ABSL_FLAG(string, input, "", "Input, if possible.");

int main(int argc, char** argv) {
  init(argc, argv);

  // SECT: Load flags
  nx = absl::GetFlag(FLAGS_nx);
  tend = absl::GetFlag(FLAGS_tend);
  periodic = absl::GetFlag(FLAGS_periodic);
  problem = absl::GetFlag(FLAGS_problem);
  flux_type = absl::GetFlag(FLAGS_flux_type);
  input = absl::GetFlag(FLAGS_input);

  initialize();
  scalar t_current = 0;
  scalar dx = (x_max - x_min) / nx;
  scalar dt = cfl(dx);

  AX_LOG(INFO) << "nx: " << nx << " tend: " << tend << " periodic: " << periodic << " dx: " << dx
               << " dt: " << dt << " cfl: " << cfl(dx);
  AX_LOG(INFO) << "Problem: " << problem << " Flux: " << flux_type;
  while (t_current < tend) {
    dt = min(cfl(dx), tend - t_current);
    scalar dtdx = dt / dx;
    rk3(dtdx);
    t_current += dt;
  }

  string out = absl::GetFlag(FLAGS_out);
  AX_CHECK_OK(write_npy_v10(out + "_density.npy", matxxr(density)));
  AX_CHECK_OK(write_npy_v10(out + "_momentum.npy", matxxr(momentum)));
  AX_CHECK_OK(write_npy_v10(out + "_energy.npy", matxxr(energy)));
  return EXIT_SUCCESS;
}
