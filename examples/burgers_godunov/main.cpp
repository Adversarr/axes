#include <absl/flags/flag.h>
#include <imgui.h>
#include <implot.h>

#include <ax/math/common.hpp>
#include <cmath>

#include "ax/core/logging.hpp"
#include "ax/core/entt.hpp"
#include "ax/core/init.hpp"
#include "ax/gl/context.hpp"
#include "ax/gl/utils.hpp"
#include "ax/math/functional.hpp"
#include "ax/math/io.hpp"
using namespace ax;

// solves burgers' eqn:
//    u_t + (1/2 u^2)_x = 0
// with
//    u(x, t = 0) = sin(x) + 1/2 x
math::RealField1 current;
Index Nx, Nt;
Real dx;

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

Real initial_data(Real x, Real dx) {
  return 0.5 * (math::sin(x) + math::sin(x + dx)) + 0.5;
}

Real cfl() {
  // Determine delta t: 
  // alpha Delta t <= Delta x / 2
  // alpha = max |f'| = max |u| = 1.5
  return 0.5 * dx / 1.5;
  // return 0.5 * dx;
}

math::RealField1 step() {
  math::RealField1 next = current;
  Real dtdx = cfl() / dx;
  for (Index i = 0; i < Nx; ++i) {
    Real u_l = current(i);
    Real u_r = current((i + 1) % Nx);
    Real u_l_next = current((i - 1 + Nx) % Nx);
    next(i) = u_l - dtdx * (godunuv_flux(u_l, u_r) - godunuv_flux(u_l_next, u_l));
  }
  return next;
}

void render(gl::UiRenderEvent) {
  if (ImGui::Begin("Burgers' Godunov")) {
    if (ImGui::Button("Step")) {
      current = step();
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset")) {
      for (Index i = 0; i < Nx; ++i) {
        current(i) = initial_data(i * dx, dx);
      }
    }
    math::Field<float, 1> current_f = current.cast<float>();
    if (ImPlot::BeginPlot("u")) {
      ImPlot::PlotLine("u", current_f.data(), current.size());
      ImPlot::EndPlot();
    }
  }
  ImGui::End();
}

ABSL_FLAG(Index, Nx, 100, "Number of grid points");
ABSL_FLAG(Index, Nt, 100, "Number of time steps");
ABSL_FLAG(Real, T, 1, "Final time");
ABSL_FLAG(bool, render, false, "Render the simulation");

int main(int argc, char** argv) {
  ax::gl::init(argc, argv);
  Nx = absl::GetFlag(FLAGS_Nx);
  Real T = absl::GetFlag(FLAGS_T);
  dx = 2 * math::pi<Real> / Nx;
  Nt = round(T / cfl());
  math::RealMatrixX u_x_t(Nx, Nt+1);
  AX_LOG(INFO) << "Delta x: " << dx;
  AX_LOG(INFO) << "CFL: " << cfl();
  current = math::RealField1(Nx);
  for (Index i = 0; i < Nx; ++i) {
    current(i) = initial_data(i * dx, dx);
  }
  u_x_t.col(0) = current.transpose();
  for (Index t = 1; t <= Nt; ++t) {
    current = step();
    u_x_t.col(t) = current.transpose();
  }
  if (absl::GetFlag(FLAGS_render)) {
    connect<gl::UiRenderEvent, &render>();
    AX_CHECK_OK(gl::enter_main_loop());
  }

  std::string path;
  path = "u" + std::to_string(Nx) + "_" + std::to_string(Index(10*T)) + ".npy";

  (math::write_npy_v10(path, u_x_t));

  ax::clean_up();
  return 0;
}