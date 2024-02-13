/**
 * @brief This is a simple example of solving the Poisson equation using the finite element method.
 *        The equation is:
 *            -Laplace U = F.
 *        Where U = (1-x) sin x (1-y) sin y
 * @date 2024-02-10
 *
 */

#include "axes/core/entt.hpp"
#include "axes/core/init.hpp"
#include "axes/fem/elements/p1.hpp"
#include "axes/gl/context.hpp"
#include "axes/gl/primitives/height_field.hpp"
#include "axes/math/linsys/sparse/LDLT.hpp"

using namespace ax;
using namespace ax::math;
idx n = 100;

bool is_diriclet(idx id) {
  idx i = id / n;
  idx j = id % n;
  return i == 0 || i == n - 1 || j == 0 || j == n - 1;
}

field3i make_triangles(idx nx, idx ny) {
  field3i triangles(3, 2 * (nx - 1) * (ny - 1));
  idx id = 0;
  for (idx i = 0; i < nx - 1; ++i) {
    for (idx j = 0; j < ny - 1; ++j) {
      idx idx00 = i * ny + j;
      idx idx01 = i * ny + j + 1;
      idx idx10 = (i + 1) * ny + j;
      idx idx11 = (i + 1) * ny + j + 1;
      if (j % 2 == 0) {
        triangles.col(id++) = vec3i{idx00, idx11, idx01};
        triangles.col(id++) = vec3i{idx00, idx11, idx10};
      } else {
        triangles.col(id++) = vec3i{idx00, idx01, idx10};
        triangles.col(id++) = vec3i{idx01, idx11, idx10};
      }
    }
  }
  return triangles;
}

int main(int argc, char** argv) {
  init(argc, argv);
  fem::P1Element2D element;
  mat3r pfpx_pgpy = math::zeros<3, 3>();
  for (idx i = 0; i < 3; ++i) {
    for (idx j = 0; j < 3; ++j) {
      pfpx_pgpy(i, j) += element.Integrate_PF_PF(i, j, 0, 0);
      pfpx_pgpy(i, j) += element.Integrate_PF_PF(i, j, 1, 1);
    }
  }
  AX_LOG(INFO) << "Local Laplacian Stiffness: \n" << pfpx_pgpy;

  // SECT: Make a mesh grid:
  math::field2r vertices(2, n * n);
  math::field3i faces = make_triangles(n, n);

  for (idx i = 0; i < n; ++i) {
    for (idx j = 0; j < n; ++j) {
      vertices.col(i * n + j) = math::vec2r{i / real(n - 1), j / real(n - 1)};
    }
  }

  // SECT: Make a P1 element:
  sp_coeff_list coefficients;
  for (auto elem : each(faces)) {
    idx idx00 = elem[0];
    idx idx01 = elem[1];
    idx idx10 = elem[2];
    fem::P1Element2D element({
        vertices.col(idx00),
        vertices.col(idx01),
        vertices.col(idx10),
    });
    for (idx i = 0; i < 3; ++i) {
      for (idx j = 0; j < 3; ++j) {
        coefficients.push_back(
            sp_coeff(elem[i], elem[j],
                     element.Integrate_PF_PF(i, j, 0, 0) + element.Integrate_PF_PF(i, j, 1, 1)));
      }
    }
  }

  // SECT: Set Dirichlet BC to 1
  sp_coeff_list coef_no_dirichlet;
  for (auto trip : coefficients) {
    if (!is_diriclet(trip.row()) && !is_diriclet(trip.col())) {
      coef_no_dirichlet.push_back(trip);
    }
  }
  for (idx i = 0; i < n; ++i) {
    for (idx j = 0; j < n; ++j) {
      idx idx00 = i * n + j;
      if (is_diriclet(idx00)) {
        coef_no_dirichlet.push_back(sp_coeff(idx00, idx00, 1));
      }
    }
  }

  vecxr b(n * n);
  b.setZero();
  for (idx i = 0; i < n; ++i) {
    for (idx j = 0; j < n; ++j) {
      idx idx00 = i * n + j;
      real x = vertices(0, idx00);
      real y = vertices(1, idx00);
      b[idx00] = -2 * (-1 + x) * cos(y) * sin(x)
                 - 2 * (-1 + y) * (cos(x) + sin(x) - x * sin(x)) * sin(y);
    }
  }

  for (idx i = 0; i < n; ++i) {
    for (idx j = 0; j < n; ++j) {
      idx idx00 = i * n + j;
      if (i == 0 || i == n - 1 || j == 0 || j == n - 1) {
        b[idx00] = 0;
      }
    }
  }

  b = b * (1.0 / ((n - 1) * (n - 1)));

  // SECT: Solve the linear system:
  sp_matxxr A(n * n, n * n);
  A.setFromTriplets(coef_no_dirichlet.begin(), coef_no_dirichlet.end());
  A.makeCompressed();
  SparseSolver_LDLT ldlt;
  LinsysProblem_Sparse problem(A, b);
  auto solution = ldlt.SolveProblem(problem);
  if (!solution.ok()) {
    AX_LOG(ERROR) << "Failed to solve the linear system";
    clean_up();
    return 1;
  }
  vecxr x = solution->solution_;
  vecxr accurate = b;
  for (idx i = 0; i < n; ++i) {
    for (idx j = 0; j < n; ++j) {
      idx idx00 = i * n + j;
      real x = vertices(0, idx00);
      real y = vertices(1, idx00);
      accurate[idx00] = (1 - x) * (1 - y) * sin(x) * sin(y);
    }
  }
  real err = (x - accurate).norm() / (n * n);
  AX_LOG(INFO) << "Error: " << err << std::boolalpha << "\tConverged? " << (err < 1e-4);

  auto& ctx = add_resource<gl::Context>();
  auto height_field= gl::make_height_field(x, n, n);
  auto ent = create_entity();
  AX_CHECK_OK(height_field);
  auto& mesh = add_component<gl::Mesh>(ent, height_field.value());
  mesh.flush_ = true;

  while (! ctx.GetWindow().ShouldClose()) {
    AX_CHECK_OK(ctx.TickLogic()); 
    AX_CHECK_OK(ctx.TickRender());
  }

  erase_resource<gl::Context>();
  clean_up();
  return 0;
}