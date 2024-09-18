#include "ax/fem/terms/mass.hpp"

#include <doctest/doctest.h>

#include "ax/core/buffer/eigen_support.hpp"
#include "ax/fem/elements/p1.hpp"
#include "ax/geometry/primitives.hpp"
using namespace ax;
using namespace ax::fem;

std::shared_ptr<Mesh> create_cube() {
  auto result = std::make_shared<Mesh>(3, 4, BufferDevice::Host);
  auto tet_cube = geo::tet_cube(1, 2, 2, 2);
  auto v = tet_cube.vertices_;
  auto i = tet_cube.indices_;
  auto e = i.cast<size_t>().eval();

  result->SetData(view_from_matrix(v), view_from_matrix(e));
  return result;
}

TEST_CASE("Mass 3D") {
  auto state = std::make_shared<State>(1, 4, BufferDevice::Host);
  auto mesh = create_cube();
  MassTerm term(state, mesh);
  term.SetDensity(1);

  auto computed = term.GetHessian().ToSparseMatrix();
  computed.makeCompressed();

  auto [v, e] = make_view(mesh->GetVertices(), mesh->GetElements());

  math::RealSparseCOO coo;
  for (size_t elem = 0; elem < mesh->GetNumElements(); ++elem) {
    std::array<math::RealVector3, 4> vert;
    for (size_t i = 0; i < 4; ++i) {
      using MapT = Eigen::Map<const math::RealVector3>;
      vert[i] = MapT(v.Offset(0, e(i, elem)));
    }

    fem::elements::P1Element<3> p1(vert);
    math::RealMatrix4 element_mass;
    for (size_t i = 0; i < 4; ++i) {
      for (size_t j = 0; j < 4; ++j) {
        element_mass(i, j) = p1.Integrate_F_F(i, j);
      }
    }

    for (size_t i = 0; i < 4; ++i) {
      for (size_t j = 0; j < 4; ++j) {
        coo.push_back(math::RealSparseEntry(e(i, elem), e(j, elem), element_mass(i, j)));
      }
    }
  }

  auto expected = math::make_sparse_matrix(mesh->GetNumVertices(), mesh->GetNumVertices(), coo);
  expected.makeCompressed();

  for (int k = 0; k < expected.outerSize(); ++k) {
    for (math::RealSparseMatrix::InnerIterator it(expected, k); it; ++it) {
      auto i = it.row();
      auto j = it.col();
      auto expected_val = it.value();
      auto computed_val = computed.coeff(i, j);
      CHECK(expected_val == doctest::Approx(computed_val).epsilon(1e-6));
    }
  }
}