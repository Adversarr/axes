#include "ax/fem/elasticity/stvk.hpp"
#include <doctest/doctest.h>

#include "ax/geometry/io.hpp"
#define AX_ELASTICITY_IMPL
#include "ax/fem/elasticity.hpp"
#include "ax/fem/mass_matrix.hpp"
#include "ax/utils/asset.hpp"

using namespace ax;
using namespace ax::fem;
using namespace ax::math;

TEST_CASE("stress") {
  auto [vert, triangle] = geo::read_obj(utils::get_asset("/mesh/obj/square_naive.obj"));
  auto mesh = std::make_shared<fem::TriMesh<2>>();
  mesh->SetMesh(triangle, vert.topRows<2>());
  auto elastic = fem::ElasticityCompute_CPU<2, elasticity::StVK>(mesh);
  math::RealVector2 lame = {1.0, 1.0};
  elastic.Update(mesh->GetVertices(), ax::fem::ElasticityUpdateLevel::Hessian);
  auto stress = elastic.Stress(lame);
  for (auto const& s : stress) {
    CHECK(doctest::Approx(s.norm()) == 0.0);
  }
  auto force = elastic.GatherStress(stress);
  for (auto const& f : math::each(force)) {
    CHECK(doctest::Approx(f.norm()) == 0.0);
  }
}

TEST_CASE("Hessian") {
  auto [vert, triangle] = geo::read_obj(utils::get_asset("/mesh/obj/square_naive.obj"));
  auto mesh = std::make_shared<fem::TriMesh<2>>();
  mesh->SetMesh(triangle, vert.topRows<2>());
  auto stress = fem::ElasticityCompute_CPU<2, elasticity::StVK>(mesh);
  math::RealVector2 lame = {1.0, 1.0};
  stress.Update(mesh->GetVertices(),ax::fem::ElasticityUpdateLevel::Hessian);
  CHECK(doctest::Approx(stress.Energy(lame).sum()) == 0);
}
