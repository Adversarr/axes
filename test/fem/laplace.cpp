#include "ax/fem/terms/laplace.hpp"

#include <doctest/doctest.h>

#include "ax/core/buffer/eigen_support.hpp"
#include "ax/fem/elements/p1.hpp"
#include "ax/geometry/primitives.hpp"
using namespace ax;
using namespace ax::fem;

static std::shared_ptr<Mesh> create_cube() {
  auto tet_cube = geo::tet_cube(1, 2, 2, 2);
  auto v = tet_cube.vertices_;
  auto i = tet_cube.indices_;
  auto e = i.cast<size_t>().eval();

  auto result = std::make_shared<Mesh>(3, 4, BufferDevice::Host);
  result->SetData(view_from_matrix(v), view_from_matrix(e));
  return result;
}

TEST_CASE("Laplace 3D") {
  auto mesh = create_cube();
  auto state = std::make_shared<State>(1, mesh->GetNumVertices(), BufferDevice::Host);
  LaplaceTerm term(state, mesh);
  term.SetDiffusivity(1);

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
    math::RealMatrix4 local;
    local.setZero();
    for (size_t i = 0; i < 4; ++i) {
      for (size_t j = 0; j < 4; ++j) {
        for (size_t k = 0; k < 3; ++k) {
          local(i, j) += p1.Integrate_PF_PF(i, j, k, k);
        }
      }
    }

    for (size_t i = 0; i < 4; ++i) {
      for (size_t j = 0; j < 4; ++j) {
        coo.emplace_back(static_cast<math::SparseIndex>(e(i, elem)),
                         static_cast<math::SparseIndex>(e(j, elem)), local(i, j));
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



TEST_CASE("Laplace 2d embed") {
  auto plane = geo::plane(1, 1, 3, 3);
  auto v = plane.vertices_;
  auto i = plane.indices_;

  auto state = std::make_shared<State>(1, v.cols(), BufferDevice::Host);
  auto mesh = std::make_shared<Mesh>(3, 3, BufferDevice::Host);
  auto e = i.cast<size_t>().eval();

  mesh->SetData(view_from_matrix(v), view_from_matrix(e));

  LaplaceTerm term(state, mesh);
  term.SetDiffusivity(1);

  auto computed = term.GetHessian().ToSparseMatrix();
  computed.makeCompressed();

  auto state2 = std::make_shared<State>(1, v.cols(), BufferDevice::Host);
  auto mesh2 = std::make_shared<Mesh>(2, 3, BufferDevice::Host);
  auto v2 = v.topRows(2).eval();

  mesh2->SetData(view_from_matrix(v2), view_from_matrix(e));

  LaplaceTerm term2(state2, mesh2);
  term2.SetDiffusivity(1);

  auto computed2 = term2.GetHessian().ToSparseMatrix();
  computed2.makeCompressed();

  // std::cout << computed << std::endl;
  // std::cout << computed2 << std::endl;

  // expect same.
  CHECK(computed.rows() == computed2.rows());
  CHECK(computed.cols() == computed2.cols());

  math::for_each_entry(computed, [&](math::SparseIndex i, math::SparseIndex j, Real val) {
    CHECK(computed2.coeff(i, j) == doctest::Approx(val).epsilon(1e-6));
  });
}