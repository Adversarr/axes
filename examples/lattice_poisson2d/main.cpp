#include <absl/flags/flag.h>
#include "axes/core/entt.hpp"
#include "axes/core/init.hpp"
#include "axes/gl/primitives/height_field.hpp"
#include "axes/gl/utils.hpp"
#include "axes/pde/poisson/lattice.hpp"
#include "axes/math/init.hpp"

using namespace ax;
idx N = 10;
real dx = 1.0 / N;
ABSL_FLAG(int, N, 10, "Number of cells in each direction");

// Example Log [x^2 + y^2] in [1, 2]x[1, 2]
// Rhs = 0.
// grad= [2x, 2y]/(x^2 + y^2)
real exact_solution(real x, real y) { return 0.5 * (x * x - y * y); }

real grad_x_exact_solution(real x, real y) { return x; }

real grad_y_exact_solution(real x, real y) { return -y; }

int main(int argc, char* argv[]) {
  ax::gl::init(argc, argv);
  math::init_parallel();

  N = absl::GetFlag(FLAGS_N);
  dx = 1.0 / N;
  pde::PoissonProblemCellCentered<2> problem(N, dx);

  math::Lattice<2, pde::PoissonProblemCellType> domain(N, N);
  math::Lattice<2, real> f(N, N);
  f = 0;
  domain = pde::PoissonProblemCellType::kInterior;
  problem.SetSource(f);
  problem.SetDomain(domain);

  math::StaggeredLattice<2, pde::PoissonProblemBoundaryType> bd_t(N, N);
  math::StaggeredLattice<2, real> bd_v(N, N);

  bd_t.X() = pde::PoissonProblemBoundaryType::kInvalid;
  bd_t.Y() = pde::PoissonProblemBoundaryType::kInvalid;
  bd_v.X() = 0;
  bd_v.Y() = 0;
  // Left & Right bd:
  for (idx j = 0; j < N; ++j) {
    bd_t.X()(0, j) = pde::PoissonProblemBoundaryType::kDirichlet;  // x = 0
    bd_t.X()(N, j) = pde::PoissonProblemBoundaryType::kDirichlet;  // x = 1
    bd_v.X()(0, j) = exact_solution(0, (0.5 + j) * dx);
    bd_v.X()(N, j) = exact_solution(1, (0.5 + j) * dx);
  }
  // Top & Bottom bd:
  for (idx i = 0; i < N; ++i) {
    bd_t.Y()(i, 0) = pde::PoissonProblemBoundaryType::kNeumann;  // y = 0
    bd_t.Y()(i, N) = pde::PoissonProblemBoundaryType::kNeumann;  // y = 1
    bd_v.Y()(i, 0) = -grad_y_exact_solution((0.5 + i) * dx, 0);
    bd_v.Y()(i, N) = grad_y_exact_solution((0.5 + i) * dx, 1);
  }

  problem.SetBoundaryCondition(bd_t, bd_v);
  AX_CHECK_OK(problem.CheckAvailable());

  auto solution = problem.Solve();
  AX_CHECK_OK(solution);
  auto sol = *solution;
  real l2_err = 0;
  for (auto const& sub : sol.Iterate()) {
    if (sub.minCoeff() > 0 && (sub.array() - N).maxCoeff() <= 0) {
      real x = (sub[0] + 0.5) * dx;
      real y = (sub[1] + 0.5) * dx;
      real exact = exact_solution(x, y);
      l2_err += std::pow(sol(sub) - exact, 2) * dx * dx;
    }
  }

  l2_err = std::sqrt(l2_err);
  AX_LOG(WARNING) << "L2 error: " << l2_err << std::endl;


  // Visualize the solution
  auto ent = create_entity();
  math::vecxr val = sol.ToMatrix().reshaped();
  auto height_field = gl::make_height_field(val, N, N);
  AX_CHECK_OK(height_field);
  auto& mesh = add_component<gl::Mesh>(ent, height_field.value());
  mesh.flush_ = true;
  AX_CHECK_OK(gl::enter_main_loop());
  clean_up();
  return 0;
}