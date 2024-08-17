#include <absl/flags/flag.h>
#include "ax/core/entt.hpp"
#include "ax/core/init.hpp"
#include "ax/gl/primitives/height_field.hpp"
#include "ax/gl/utils.hpp"
#include "ax/fdfv/poisson/lattice.hpp"
#include "ax/math/init.hpp"

using namespace ax;
Index N = 10;
real dx = 1.0 / N;
ABSL_FLAG(int, N, 10, "Number of cells in each direction");

// Example Log [x^2 + y^2] in [1, 2]x[1, 2]
// Rhs = 0.
// grad= [2x, 2y]/(x^2 + y^2)
real exact_solution(real x, real y) { return 0.5 * (x * x - y * y); }

real grad_x_exact_solution(real x, real /*y*/) { return x; }

real grad_y_exact_solution(real /*x*/, real y) { return -y; }

int main(int argc, char* argv[]) {
  ax::gl::init(argc, argv);
  math::init_parallel();

  N = absl::GetFlag(FLAGS_N);
  dx = 1.0 / N;
  pde::PoissonProblemCellCentered<2> problem(N, dx);

  math::Lattice<2, pde::PoissonProblemCellType> domain({N, N});
  math::Lattice<2, real> f({N, N});
  f = 0;
  domain = pde::PoissonProblemCellType::kInterior;
  problem.SetSource(f);
  problem.SetDomain(domain);


  math::Lattice<2, std::array<pde::PoissonProblemBoundaryType, 2>> bd_t({N, N}, math::staggered);
  math::Lattice<2, math::RealVector2> bd_v({N, N}, math::staggered);
  bd_v = math::zeros<2>();

  for (auto& t: bd_t) {
    t[0] = pde::PoissonProblemBoundaryType::kInvalid;
    t[1] = pde::PoissonProblemBoundaryType::kInvalid;
  }

  // Left & Right bd:
  for (Index j = 0; j < N; ++j) {
    bd_t(0, j)[0] = pde::PoissonProblemBoundaryType::kDirichlet;  // x = 0
    bd_t(N, j)[0] = pde::PoissonProblemBoundaryType::kDirichlet;  // x = 1
    bd_v(0, j)[0] = exact_solution(0, (0.5 + j) * dx);
    bd_v(N, j)[0] = exact_solution(1, (0.5 + j) * dx);
  }
  // Top & Bottom bd:
  for (Index i = 0; i < N; ++i) {
    bd_t(i, 0)[1] = pde::PoissonProblemBoundaryType::kNeumann;  // y = 0
    bd_t(i, N)[1] = pde::PoissonProblemBoundaryType::kNeumann;  // y = 1
    bd_v(i, 0)[1] = -grad_y_exact_solution((0.5 + i) * dx, 0);
    bd_v(i, N)[1] = grad_y_exact_solution((0.5 + i) * dx, 1);
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
  math::RealVectorX val = sol.ToMatrix().reshaped();
  auto height_field = gl::make_height_field(val, N, N);
  AX_CHECK_OK(height_field);
  auto& mesh = add_component<gl::Mesh>(ent, height_field.value());
  mesh;
  AX_CHECK_OK(gl::enter_main_loop());
  clean_up();
  return 0;
}