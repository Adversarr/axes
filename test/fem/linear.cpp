#include "axes/pde/elasticity/linear.hpp"
#include <doctest/doctest.h>

#include "axes/geometry/io.hpp"
#define AX_ELASTICITY_IMPL
#include "axes/pde/fem/deform.hpp"
#include "axes/pde/fem/elasticity.hpp"
#include "axes/pde/fem/mass_matrix.hpp"
#include "axes/pde/fem/p1mesh.hpp"

#include "axes/utils/asset.hpp"

using namespace ax;
using namespace ax::pde;
using namespace ax::math;

std::unique_ptr<fem::P1Mesh<2>> make_square(idx n_div) {
  auto mesh = std::make_unique<fem::P1Mesh<2>>();
  field2r vertices;
  vertices.resize(2, n_div * n_div);
  field3i elements;
  elements.resize(3, 2 * (n_div - 1) * (n_div - 1));

  for (idx i = 0; i < n_div; ++i) {
    for (idx j = 0; j < n_div; ++j) {
      vertices(0, i * n_div + j) = i / real(n_div - 1);
      vertices(1, i * n_div + j) = j / real(n_div - 1);
    }
  }

  for (idx i = 0; i < n_div - 1; ++i) {
    for (idx j = 0; j < n_div - 1; ++j) {
      elements(0, 2 * (i * (n_div - 1) + j) + 0) = i * n_div + j;
      elements(1, 2 * (i * (n_div - 1) + j) + 0) = i * n_div + j + 1;
      elements(2, 2 * (i * (n_div - 1) + j) + 0) = (i + 1) * n_div + j;

      elements(0, 2 * (i * (n_div - 1) + j) + 1) = i * n_div + j + 1;
      elements(1, 2 * (i * (n_div - 1) + j) + 1) = (i + 1) * n_div + j + 1;
      elements(2, 2 * (i * (n_div - 1) + j) + 1) = (i + 1) * n_div + j;
    }
  }

  AX_CHECK_OK(mesh->SetMesh(elements, vertices));
  return mesh;
}

TEST_CASE("mass2d") {
  auto [vert, triangle] = geo::read_obj(utils::get_asset("/mesh/obj/square_naive.obj")).value();
  auto mesh = std::make_unique<fem::P1Mesh<2>>();
  AX_CHECK_OK(mesh->SetMesh(triangle, vert.topRows<2>()));
  // auto mesh = make_square(3);
  auto mass_compute = fem::MassMatrixCompute<2>(*mesh);
  auto result = mass_compute(1.0);
  real sum = 0;
  for (auto ijv : result) {
    // std::cout << ijv.row() << " " << ijv.col() << " " << ijv.value() << std::endl;
    sum += ijv.value();
  }
  CHECK(sum == doctest::Approx(1.0));
}

TEST_CASE("stress") {
  auto [vert, triangle] = geo::read_obj(utils::get_asset("/mesh/obj/square_naive.obj")).value();
  auto mesh = std::make_unique<fem::P1Mesh<2>>();
  AX_CHECK_OK(mesh->SetMesh(triangle, vert.topRows<2>()));
  fem::Deformation<2> deform(*mesh, vert.topRows<2>());
  auto def = deform.Forward();
  auto elastic = fem::ElasticityCompute<2, elasticity::Linear>(deform);
  math::vec2r lame = {1.0, 1.0};
  elastic.UpdateDeformationGradient();
  auto stress = elastic.Stress(lame);
  for (auto const& s : stress) {
    CHECK(doctest::Approx(s.norm()) == 0.0);
  }
  auto force = deform.StressToForce(stress);
  for (auto const& f : math::each(force)) {
    CHECK(doctest::Approx(f.norm()) == 0.0);
  }
}

TEST_CASE("Hessian") {
  auto [vert, triangle] = geo::read_obj(utils::get_asset("/mesh/obj/square_naive.obj")).value();
  auto mesh = std::make_unique<fem::P1Mesh<2>>();
  AX_CHECK_OK(mesh->SetMesh(triangle, vert.topRows<2>()));
  fem::Deformation<2> deform(*mesh, vert.topRows<2>());
  auto stress = fem::ElasticityCompute<2, elasticity::Linear>(deform);
  math::vec2r lame = {1.0, 1.0};
  stress.UpdateDeformationGradient();
  for (auto const& s : stress.Hessian(lame)) {
    real s00 = s(0, 0);
    real s11 = s(1, 1);
    CHECK(doctest::Approx(s00) == 3 * s11);
    CHECK(doctest::Approx(s(3, 3)) == s00);
    CHECK(doctest::Approx(s(2, 2)) == s11);
  }

  CHECK(doctest::Approx(stress.Energy(lame)) == 0);
}