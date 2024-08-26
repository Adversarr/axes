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

#include "ax/core/init.hpp"
#include "ax/fem/elements/p1.hpp"
#include "ax/fem/laplace_matrix.hpp"
#include "ax/fem/trimesh.hpp"
#include "ax/geometry/primitives.hpp"
#include "ax/geometry/topology.hpp"
#include "ax/math/io.hpp"
#include "ax/math/linsys/preconditioner/Identity.hpp"
#include "ax/math/linsys/preconditioner/IncompleteCholesky.hpp"
#include "ax/math/linsys/preconditioner/IncompleteLU.hpp"
#include "ax/math/linsys/sparse/ConjugateGradient.hpp"
#include "ax/math/linsys/sparse/LDLT.hpp"
#include "ax/utils/asset.hpp"
#include "ax/utils/ndrange.hpp"

using namespace ax;
using namespace ax::math;

Real neg_laplace(Real x, Real y, Real z) {
  return 2 * (-1 + x) * (-1 + y) * cos(z) * sin(x) * sin(y)
         + 2 * (-1 + x) * (-1 + z) * cos(y) * sin(x) * sin(z)
         + 2 * (-1 + y) * (-1 + z) * cos(x) * sin(y) * sin(z)
         - 3 * (-1 + x) * (-1 + y) * (-1 + z) * sin(x) * sin(y) * sin(z);
}

Real analytical(Real x, Real y, Real z) {
  return (1 - x) * sin(x) * (1 - y) * sin(y) * (1 - z) * sin(z);
}

void print_basic_test_case() {
  fem::elements::P1Element3D element;
  RealMatrix4 pfpx_pgpy = math::zeros<4, 4>();
  for (Index i = 0; i < 4; ++i) {
    for (Index j = 0; j < 4; ++j) {
      pfpx_pgpy(i, j) += element.Integrate_PF_PF(i, j, 0, 0);
      pfpx_pgpy(i, j) += element.Integrate_PF_PF(i, j, 1, 1);
      pfpx_pgpy(i, j) += element.Integrate_PF_PF(i, j, 2, 2);
    }
  }
  AX_LOG(INFO) << "Local Laplacian Stiffness: \n" << pfpx_pgpy;

  math::RealField3 vertices(3, 4);
  vertices.col(0) = RealVector3{0, 0, 0};
  vertices.col(1) = RealVector3{1, 0, 0};
  vertices.col(2) = RealVector3{0, 1, 0};
  vertices.col(3) = RealVector3{0, 0, 1};

  vertices *= 0.1;
  math::IndexField4 elements(4, 1);
  elements.col(0) = IndexVector4{0, 1, 2, 3};

  std::shared_ptr<fem::TriMesh<3>> pmesh = std::make_shared<fem::TriMesh<3>>();
  pmesh->SetMesh(elements, vertices);

  math::RealSparseMatrix K = fem::LaplaceMatrixCompute<3>(*pmesh)(1.0);
  std::cout << K.toDense() << std::endl;
}

int main(int argc, char** argv) {
  initialize(argc, argv);
  print_basic_test_case();
  math::RealMatrixX input_mesh_vertices
      = math::read_npy_v10_real(utils::get_asset("/mesh/npy/beam_high_res_vertices.npy"));
  math::IndexMatrixX input_mesh_elements
      = math::read_npy_v10_Index(utils::get_asset("/mesh/npy/beam_high_res_elements.npy"));

  math::RealField3 vertices = input_mesh_vertices.transpose();
  math::IndexField4 elements = input_mesh_elements.transpose();
  vertices.row(0) /= vertices.row(0).maxCoeff();

  // Index nx = 5;
  // auto [vertices, elements] = geo::tet_cube(1, nx, nx, nx);

  vertices = vertices.array() * 0.5 + 0.5;
  Index const nVertices = vertices.cols();

  std::shared_ptr<fem::TriMesh<3>> pmesh = std::make_shared<fem::TriMesh<3>>();
  std::set<int> dirichlet;
  for (auto i : utils::range(nVertices)) {
    // [0, 1] x [0, 1] x [0, 1]
    if (vertices(0, i) == 0 || vertices(0, i) == 1 || vertices(1, i) == 0 || vertices(1, i) == 1
        || vertices(2, i) == 0 || vertices(2, i) == 1) {
      dirichlet.insert(i);
    }
  }

  std::cout << "Dirichlet boundary: " << dirichlet.size() << std::endl;

  std::cout << "min x: " << vertices.row(0).minCoeff() << " max x: " << vertices.row(0).maxCoeff()
            << std::endl;
  std::cout << "min y: " << vertices.row(1).minCoeff() << " max y: " << vertices.row(1).maxCoeff()
            << std::endl;
  std::cout << "min z: " << vertices.row(2).minCoeff() << " max z: " << vertices.row(2).maxCoeff()
            << std::endl;

  pmesh->SetNumDofPerVertex(1);
  pmesh->SetMesh(elements, vertices);

  for (auto const& i : dirichlet) {
    // there is only one degree of freedom.
    pmesh->MarkDirichletBoundary(i, 0, 0);
  }

  math::RealSparseMatrix K = fem::LaplaceMatrixCompute<3>(*pmesh)(1.0);
  pmesh->FilterMatrixDof(0, K);
  K.makeCompressed();
  math::RealVectorX b(nVertices);
  math::RealVectorX accurate(nVertices);
  b.setZero();

  for (auto const& e : math::each(elements)) {
    math::RealVector3 x0 = vertices.col(e[0]);
    math::RealVector3 x1 = vertices.col(e[1]);
    math::RealVector3 x2 = vertices.col(e[2]);
    math::RealVector3 x3 = vertices.col(e[3]);
    Real volume = (x1 - x0).cross(x2 - x0).dot(x3 - x0);
    volume = fabs(volume) / 6.0;

    for (auto const& i : e) {
      math::RealVector3 xi = vertices.col(i);
      Real load = neg_laplace(xi(0), xi(1), xi(2));
      b(i) += load * volume / 4;
    }
  }

  for (auto const& i : utils::range(nVertices)) {
    auto const& v = vertices.col(i);
    accurate(i) = analytical(v(0), v(1), v(2));
    if (dirichlet.find(i) != dirichlet.end()) {
      b(i) = accurate(i);
    }
  }

  SparseSolver_ConjugateGradient spsolve;
  spsolve.SetPreconditioner(std::make_unique<Preconditioner_IncompleteLU>());
  spsolve.SetOptions({{"max_iter", 10000}});
  spsolve.SetProblem(K).Compute();

  auto result = spsolve.Solve(b, {});
  RealVectorX x = result.solution_;
  RealVectorX error = x - accurate;
  std::cout << "num vertices: " << nVertices << std::endl;

  Real l2_error = error.squaredNorm() / nVertices;

  AX_LOG(INFO) << "L2 error: " << l2_error;
  clean_up();
  return 0;
}
