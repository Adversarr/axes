#include <absl/flags/flag.h>
#include <cmath>
#include <imgui.h>
#include <implot.h>

#include <ax/math/common.hpp>

#include "ax/core/echo.hpp"
#include "ax/core/init.hpp"
#include "ax/math/functional.hpp"
#include "ax/math/io.hpp"

using namespace ax;

real tol_var = 1e-6;
real tol_grad = 1e-6;
real alpha = 0.0;
real beta = 1.0;
idx max_iter = 100;
idx nx = 100;
math::vecxr grid;
math::vecxr result;
math::matxxr u_x_t;
real t = 0.;

ABSL_FLAG(real, tol_var, 1e-9, "Tolerance for variable convergence");
ABSL_FLAG(real, tol_grad, 1e-9, "Tolerance for gradient convergence");
ABSL_FLAG(real, alpha, 0.0, "coefficient alpha");
ABSL_FLAG(real, beta, 1.0, "coefficient beta");
ABSL_FLAG(idx, max_iter, 100, "Maximum number of iterations");
ABSL_FLAG(idx, nx, 100, "Number of grid points");
ABSL_FLAG(idx, nt, 100, "Number of time steps");
ABSL_FLAG(bool, export, false, "Export the result to a file");
ABSL_FLAG(bool, integrate, false, "Integrate the result in the computation cell");
ABSL_FLAG(real, t, 1.0, "The max simulation time.");
ABSL_FLAG(std::string, export_file, "result.npy", "The file to export the result.");


// Try to solve the 1D Burgers' equation use the method of characterics
// The problem is formulated as:
//   u_t + u u_x = 0
//   u(x, t = 0) = sin(x)
//   2 Pi periodic boundary condition

// Characteristic equation:
//  x = x0 + sin(x0) * t, solve x0
real fn(real x, real t, real x0) { return x0 + math::sin(x0) * t - x; }

real grad_fn(real /*x*/, real t, real x0) { return 1 + math::cos(x0) * t; }

real solve_x0(real x, real t) {
  // First, find out which half period the x is in
  idx half_period = math::floor(x / math::pi<real>);
  // The initial guess will be at the center of half period
  real x0 = (half_period + 0.5) * math::pi<real>;

  real left_boundary = half_period * math::pi<real>;
  real right_boundary = (half_period + 1) * math::pi<real>;
  if (x - left_boundary < tol_var) {
    return left_boundary;
  } else if (right_boundary - x < tol_var) {
    return right_boundary;
  }

  // Newton's method
  bool converged = false;
  idx iter = 0;
  do {
    real f = fn(x, t, x0);
    real g = grad_fn(x, t, x0);
    real dx = -f / g;
    real new_x = x0 + dx;
    if (new_x < left_boundary) {
      new_x = 0.5 * (x0 + left_boundary);
    } else if (new_x > right_boundary) {
      new_x = 0.5 * (x0 + right_boundary);
    }
    dx = new_x - x0;
    x0 = new_x;

    if (math::abs(dx) < tol_var) {
      converged = true;
    }
    if (iter++ > max_iter) {
      break;
    }
    if (math::abs(g) < tol_grad) {
      break;
    }

  } while (!converged);

  if (converged) {
    return x0;
  } else {
    return math::nan<real>;
  }
}

void recompute() {
  result = grid;
  for (idx i = 0; i < grid.size(); ++i) {
    real x = grid[i];
    real x0 = solve_x0(x - alpha * t, beta * t);
    if (std::isnan(x0)) {
      AX_LOG(ERROR) << "Failed to solve x0 at x = " << x;
    }
    real u = math::sin(x0) * beta + alpha;
    result[i] = u;
  }
}

int main(int argc, char** argv) {
  init(argc, argv);
  // Fetch all the flags
  tol_var = absl::GetFlag(FLAGS_tol_var);
  tol_grad = absl::GetFlag(FLAGS_tol_grad);
  max_iter = absl::GetFlag(FLAGS_max_iter);
  nx = absl::GetFlag(FLAGS_nx);
  t = absl::GetFlag(FLAGS_t);

  alpha = absl::GetFlag(FLAGS_alpha);
  beta = absl::GetFlag(FLAGS_beta);

  // Do computation
  real x_min = 0 * math::pi<real>;
  real x_max = 2 * math::pi<real>;
  grid = math::linspace(x_min, x_max, nx);
  idx nt = absl::GetFlag(FLAGS_nt);
  u_x_t.resize(nx, nt + 1);
  real dt = t / nt;
  for (idx i = 0; i <= nt; ++i) {
    t = i * dt;
    recompute();
    u_x_t.col(i) = result;
  }

  std::string export_file = absl::GetFlag(FLAGS_export_file);
  AX_LOG(INFO) << "Exporting the result to " << export_file;
  AX_CHECK_OK(math::write_npy_v10(export_file, u_x_t)) << "Failed to write to " << export_file;

  clean_up();
  return 0;
}