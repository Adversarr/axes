#include "axes/core/entt.hpp"
#include "axes/core/init.hpp"
#include "axes/gl/context.hpp"
#include "axes/gl/primitives/height_field.hpp"
#include "axes/pde/poisson/lattice.hpp"
#include <absl/flags/flag.h>
#include "axes/gl/utils.hpp"

using namespace ax;
idx N = 10;
real dx = 1.0 / N;

ABSL_FLAG(int, N, 10, "Number of cells in each direction");


int main(int argc, char* argv[]) {
  ax::gl::init(argc, argv);
  N = absl::GetFlag(FLAGS_N);
  dx = 1.0 / N;
  pde::PoissonProblemOnLattice<2> problem(N, dx);
  problem.SetC(1);

  // Example (1-x)sin(x) (1-y)sin(y)
  // RHS_f = -2 * (-1 + x) * cos(y) * sin(x)
  //  - 2 * (-1 + y) * (cos(x) + sin(x) - x * sin(x)) * sin(y);
  // Exact solution: (1-x)sin(x) (1-y)sin(y)
  auto domain = math::Lattice<2, pde::PoissonProblemCellType>(N, N);
  math::Lattice<2, real> f(N, N);
  for (auto const& sub : f.Iterate()) {
    real x = (sub[0] + 0.5) * dx;
    real y = (sub[1] + 0.5) * dx;
    f(sub) = -2 * (-1 + x) * std::cos(y) * std::sin(x) -
             2 * (-1 + y) * (std::cos(x) + std::sin(x) - x * std::sin(x)) * std::sin(y);
  }
  problem.SetSource(f);


  for (auto const& sub : domain.Iterate()) {
    domain(sub) = pde::PoissonProblemCellType::kInterior;
  }
  problem.SetDomain(domain);

  math::StaggeredLattice<2, pde::PoissonProblemBoundaryType> bd_t({N, N});
  bd_t.X() = pde::PoissonProblemBoundaryType::kInvalid;
  bd_t.Y() = pde::PoissonProblemBoundaryType::kInvalid;
  // Left & Right bd:
  for (idx j = 0; j < N; ++j) {
    bd_t.X()(0, j) = pde::PoissonProblemBoundaryType::kDirichlet;
    bd_t.X()(N, j) = pde::PoissonProblemBoundaryType::kDirichlet;
  }
  // Top & Bottom bd:
  for (idx i = 0; i < N; ++i) {
    bd_t.Y()(i, 0) = pde::PoissonProblemBoundaryType::kDirichlet;
    bd_t.Y()(i, N) = pde::PoissonProblemBoundaryType::kDirichlet;
  }

  math::StaggeredLattice<2, real> bd_v({N, N});
  bd_v.X() = 0;
  bd_v.Y() = 0;
  problem.SetBoundaryCondition(bd_t, bd_v);
  AX_CHECK_OK(problem.CheckAvailable());

  auto solution = problem.Solve();
  AX_CHECK_OK(solution);
  auto sol = *solution;
  real l2_err = 0;
  for (auto const& sub : sol.Iterate()) {
    if ((sub.minCoeff() > 0 && ((sub).array() - N).maxCoeff() <= 0)) {
      math::vecr<2> x = sub.cast<real>() * dx ;
      x.array() += 0.5 * dx;
      real exact = (1 - x[0]) * std::sin(x[0]) * (1 - x[1]) * std::sin(x[1]);
      l2_err += std::pow(sol(sub) - exact, 2) * dx * dx;
      // std::cout << "x: " << x.transpose() << " exact: " << exact << " sol: " << sol(sub) << std::endl;
    }
  }

  l2_err = std::sqrt(l2_err);
  std::cout << "L2 error: " << l2_err << std::endl;

  auto ent = create_entity();
  math::vecxr val = sol.ToMatrix().reshaped();
  auto height_field = gl::make_height_field(val, N, N);
  AX_CHECK_OK(height_field);
  auto& mesh = add_component<gl::Mesh>(ent, height_field.value());
  mesh.flush_ = true;
  auto& ctx = get_resource<gl::Context>();
  while (!ctx.GetWindow().ShouldClose()) {
    AX_CHECK_OK(ctx.TickLogic());
    AX_CHECK_OK(ctx.TickRender());
  }
  clean_up();
  return 0;
}