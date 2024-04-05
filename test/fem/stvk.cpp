#include "ax/fem/elasticity/stvk.hpp"
#include <doctest/doctest.h>

#include "ax/geometry/io.hpp"
#define AX_ELASTICITY_IMPL
#include "ax/fem/deform.hpp"
#include "ax/fem/elasticity.hpp"
#include "ax/fem/mass_matrix.hpp"
#include "ax/fem/mesh/p1mesh.hpp"

#include "ax/utils/asset.hpp"

using namespace ax;
using namespace ax::fem;
using namespace ax::math;

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
  CHECK(sum == doctest::Approx(2.0));
}

TEST_CASE("stress") {
  auto [vert, triangle] = geo::read_obj(utils::get_asset("/mesh/obj/square_naive.obj")).value();
  auto mesh = std::make_unique<fem::P1Mesh<2>>();
  AX_CHECK_OK(mesh->SetMesh(triangle, vert.topRows<2>()));
  fem::Deformation<2> deform(*mesh, vert.topRows<2>());
  auto def = deform.Forward();
  auto elastic = fem::ElasticityCompute<2, elasticity::StVK>(deform);
  math::vec2r lame = {1.0, 1.0};
  elastic.UpdateDeformationGradient();
  auto stress = elastic.Stress(lame);
  for (auto const& s : stress) {
    CHECK(doctest::Approx(s.norm()) == 0.0);
  }
  auto force = deform.StressToVertices(stress);
  for (auto const& f : math::each(force)) {
    CHECK(doctest::Approx(f.norm()) == 0.0);
  }
}

TEST_CASE("Hessian") {
  auto [vert, triangle] = geo::read_obj(utils::get_asset("/mesh/obj/square_naive.obj")).value();
  auto mesh = std::make_unique<fem::P1Mesh<2>>();
  AX_CHECK_OK(mesh->SetMesh(triangle, vert.topRows<2>()));
  fem::Deformation<2> deform(*mesh, vert.topRows<2>());
  auto stress = fem::ElasticityCompute<2, elasticity::StVK>(deform);
  math::vec2r lame = {1.0, 1.0};
  stress.UpdateDeformationGradient();
  CHECK(doctest::Approx(stress.Energy(lame).sum()) == 0);
}