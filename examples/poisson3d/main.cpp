/**
 * @brief This is a simple example of solving the Poisson equation using the finite element method.
 *        The equation is:
 *            -Laplace U = F.
 *        Where U = (1-x) sin x (1-y) sin y (1-z) sin z
 * @date 2024-02-13
 *
 */

// - laplace = 2*(-1 + x)*(-1 + y)*Cos[z]*Sin[x]*Sin[y] + 2*(-1 + x)*(-1 + z)*Cos[y]*Sin[x]*Sin[z] +
//             2*(-1 + y)*(-1 + z)*Cos[x]*Sin[y]*Sin[z] - 3*(-1 + x)*(-1 + y)*(-1 +
//             z)*Sin[x]*Sin[y]*Sin[z]

#include <absl/flags/flag.h>

#include "axes/core/init.hpp"
#include "axes/fem/elements/p1.hpp"
#include "axes/math/linsys/sparse/LDLT.hpp"

using namespace ax;
using namespace ax::math;
idx n = 10;
ABSL_FLAG(idx, n, 10, "Number of grid points in each direction.");

real neg_laplace(real x, real y, real z) {
  return 2 * (-1 + x) * (-1 + y) * cos(z) * sin(x) * sin(y)
         + 2 * (-1 + x) * (-1 + z) * cos(y) * sin(x) * sin(z)
         + 2 * (-1 + y) * (-1 + z) * cos(x) * sin(y) * sin(z)
         - 3 * (-1 + x) * (-1 + y) * (-1 + z) * sin(x) * sin(y) * sin(z);
}

real analytical(real x, real y, real z) {
  return (1 - x) * sin(x) * (1 - y) * sin(y) * (1 - z) * sin(z);
}

std::pair<field3r, field4i> tetra_grid3(idx nx, idx ny, idx nz) {
  field3r vertices(3, nx * ny * nz);
  field4i elements(4, 5 * (nx - 1) * (ny - 1) * (nz - 1));
  idx id = 0;
  for (idx i = 0; i < nx; ++i) {
    for (idx j = 0; j < ny; ++j) {
      for (idx k = 0; k < nz; ++k) {
        vertices.col(id) = vec3r{i / real(nx - 1), j / real(ny - 1), k / real(nz - 1)};
        id++;
      }
    }
  }

  id = 0;
  for (idx i = 0; i < nx - 1; ++i) {
    for (idx j = 0; j < ny - 1; ++j) {
      for (idx k = 0; k < nz - 1; ++k) {
        idx idx000 = i * ny * nz + j * nz + k;
        idx idx100 = (i + 1) * ny * nz + j * nz + k;
        idx idx010 = i * ny * nz + (j + 1) * nz + k;
        idx idx001 = i * ny * nz + j * nz + k + 1;
        idx idx110 = (i + 1) * ny * nz + (j + 1) * nz + k;
        idx idx101 = (i + 1) * ny * nz + j * nz + k + 1;
        idx idx011 = i * ny * nz + (j + 1) * nz + k + 1;
        idx idx111 = (i + 1) * ny * nz + (j + 1) * nz + k + 1;
        elements.col(id++) = vec4i{idx000, idx100, idx010, idx001};
        elements.col(id++) = vec4i{idx100, idx010, idx110, idx111};
        elements.col(id++) = vec4i{idx100, idx010, idx001, idx111};
        elements.col(id++) = vec4i{idx100, idx001, idx101, idx111};
        elements.col(id++) = vec4i{idx011, idx010, idx001, idx111};
      }
    }
  }

  return {vertices, elements};
}

bool is_dirichlet(idx id) {
  idx i = id / (n * n);
  idx j = (id % (n * n)) / n;
  idx k = id % n;
  return i == 0 || i == n - 1 || j == 0 || j == n - 1 || k == 0 || k == n - 1;
}

int main(int argc, char** argv) {
  init(argc, argv);
  n = absl::GetFlag(FLAGS_n);
  fem::P1Element3D element;
  mat4r pfpx_pgpy = math::zeros<4, 4>();
  for (idx i = 0; i < 4; ++i) {
    for (idx j = 0; j < 4; ++j) {
      pfpx_pgpy(i, j) += element.Integrate_PF_PF(i, j, 0, 0);
      pfpx_pgpy(i, j) += element.Integrate_PF_PF(i, j, 1, 1);
      pfpx_pgpy(i, j) += element.Integrate_PF_PF(i, j, 2, 2);
    }
  }
  AX_LOG(INFO) << "Local Laplacian Stiffness: \n" << pfpx_pgpy;
  auto [vertices, elements] = tetra_grid3(n, n, n);

  sp_coeff_list coefficients;
  for (auto elem : each(elements)) {
    idx idx000 = elem[0];
    idx idx100 = elem[1];
    idx idx010 = elem[2];
    idx idx110 = elem[3];
    vec3r v000 = vertices.col(idx000);
    vec3r v100 = vertices.col(idx100);
    vec3r v010 = vertices.col(idx010);
    vec3r v001 = vertices.col(idx110);
    fem::P1Element3D element({v000, v100, v010, v001});
    for (idx i = 0; i < 4; ++i) {
      for (idx j = 0; j < 4; ++j) {
        idx ei = elem[i];
        idx ej = elem[j];
        coefficients.push_back(sp_coeff(ei, ej,
                                        element.Integrate_PF_PF(i, j, 0, 0)
                                            + element.Integrate_PF_PF(i, j, 1, 1)
                                            + element.Integrate_PF_PF(i, j, 2, 2)));
      }
    }
  }

  sp_coeff_list coef_no_dirichlet;
  for (auto coef : coefficients) {
    if (!is_dirichlet(coef.row()) && !is_dirichlet(coef.col())) {
      coef_no_dirichlet.push_back(coef);
    }
  }
  for (idx i = 0; i < n * n * n; ++i) {
    if (is_dirichlet(i)) {
      coef_no_dirichlet.push_back(sp_coeff(i, i, 1));
    }
  }

  sp_matxxr K(n * n * n, n * n * n);
  K.setFromTriplets(coef_no_dirichlet.begin(), coef_no_dirichlet.end());
  K.makeCompressed();

  vecxr b(n * n * n);
  b.setZero();
  for (idx i = 1; i < n - 1; ++i) {
    for (idx j = 1; j < n - 1; ++j) {
      for (idx k = 1; k < n - 1; ++k) {
        idx idx000 = i * n * n + j * n + k;
        real x = vertices(0, idx000);
        real y = vertices(1, idx000);
        real z = vertices(2, idx000);
        b[idx000] = neg_laplace(x, y, z);
      }
    }
  }
  b *= 1.0 / (n * n * n);

  vecxr accurate(n * n * n);
  for (idx i = 0; i < n; ++i) {
    for (idx j = 0; j < n; ++j) {
      for (idx k = 0; k < n; ++k) {
        idx idx000 = i * n * n + j * n + k;
        real x = vertices(0, idx000);
        real y = vertices(1, idx000);
        real z = vertices(2, idx000);
        accurate[idx000] = analytical(x, y, z);
      }
    }
  }

  LinsysProblem_Sparse problem(K, b);
  SparseSolver_LDLT ldlt;

  auto result = ldlt.SolveProblem(problem);
  if (!result.ok()) {
    AX_LOG(ERROR) << "Failed to solve the linear system.";
  }

  vecxr x = result->solution_;
  vecxr error = x - accurate;
  real l2_error = sqrt(error.dot(K * error));

  AX_LOG(INFO) << "L2 error: " << l2_error;
  clean_up();
  return 0;
}