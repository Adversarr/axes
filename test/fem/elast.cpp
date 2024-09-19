#include <doctest/doctest.h>

#include "ax/core/buffer/copy.hpp"
#include "ax/core/buffer/eigen_support.hpp"
#include "ax/fem/elasticity/linear.hpp"
#include "ax/fem/elasticity_cpu.hpp"
#include "ax/fem/terms/elasticity.hpp"
#include "ax/geometry/primitives.hpp"
using namespace ax;
using namespace ax::fem;

static std::shared_ptr<Mesh> create_cube(BufferDevice device = BufferDevice::Host) {
  auto result = std::make_shared<Mesh>(3, 4, device);
  auto tet_cube = geo::tet_cube(1, 2, 2, 2);
  auto v = tet_cube.vertices_;
  auto i = tet_cube.indices_;
  auto e = i.cast<size_t>().eval();

  result->SetData(view_from_matrix(v), view_from_matrix(e));
  return result;
}

TEST_CASE("Elasticity") {
  auto state = std::make_shared<State>(3, 8, BufferDevice::Host);
  auto mesh = create_cube();
  ElasticityTerm term(state, mesh);
  auto lame = term.compute_.Lame()->View();
  auto u_lame = elasticity::compute_lame(1e7, 0.3);
  for (size_t i = 0; i < 5; ++i) {
    for (size_t j = 0; j < 2; ++j) {
      lame(j, i) = u_lame[j];
    }
  }

  state->GetVariables()->SetBytes(0);
  SUBCASE("Zero") {
    term.UpdateGradient();
    term.UpdateEnergy();

    auto e = term.GetEnergy();
    CHECK(doctest::Approx(e) == 0);
  }

  auto linear_mesh = std::make_shared<fem::LinearMesh<3>>();
  auto tet_cube = geo::tet_cube(1, 2, 2, 2);
  linear_mesh->SetMesh(tet_cube.indices_, tet_cube.vertices_);
  linear_mesh->SetNumDofPerVertex(3);
  ElasticityCompute_CPU<3, elasticity::Linear> gt(linear_mesh);
  gt.SetLame(u_lame);
  gt.RecomputeRestPose();

  SUBCASE("Disturb") {
    auto u = state->GetVariables()->View();
    u(0, 0) += 0.1;
    u(1, 0) += -0.1;
    u(2, 4) += 0.1;
    u(1, 2) += -0.1;
    math::RealField3 pose(3, 8);
    pose.setZero();
    auto cv = mesh->GetVertices()->ConstView();
    for (size_t i = 0; i < 8; ++i) {
      for (size_t j = 0; j < 3; ++j) {
        pose(j, i) = u(j, i) + cv(j, i);
      }
    }

    gt.Update(pose, ElasticityUpdateLevel::Hessian);

    gt.UpdateHessian(false);
    gt.UpdateStress();
    gt.UpdateEnergy();
    gt.GatherHessianToVertices();
    gt.GatherStressToVertices();
    gt.GatherEnergyToVertices();

    term.UpdateHessian();
    term.UpdateGradient();
    term.UpdateEnergy();

    // check the deformation gradient.
    auto f_gt = gt.GetDeformationGradient();
    auto f = term.compute_.DeformGrad()->View();
    for (size_t elem = 0; elem < 5; ++elem) {
      for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) {
          CHECK(doctest::Approx(f(i, j, elem)) == f_gt[elem](i, j));
        }
      }
    }

    // check energy
    auto [e, r] = make_view(term.compute_.EnergyDensity(), term.rest_volume_);
    auto gt_energy = gt.GetEnergyOnElements();
    for (size_t i = 0; i < 5; ++i) {
      auto energy = e(i) * r(i);
      CHECK(doctest::Approx(energy) == gt_energy(i));
    }

    // check gradient
    auto gt_grad = gt.GetStressOnVertices();
    auto grad = term.GetGradient();
    for (size_t i = 0; i < 8; ++i) {
      for (size_t j = 0; j < 3; ++j) {
        CHECK(doctest::Approx(grad(j, i)) == gt_grad(j, i));
      }
    }

    // check hessian
    auto gt_hess = gt.GetHessianOnVertices();
    auto hess = term.GetHessian().ToSparseMatrix();
    math::for_each_entry(gt_hess, [&](size_t i, size_t j, Real v) {
      CHECK(doctest::Approx(hess.coeff(i, j)) == v);
    });

    // math::for_each_entry(hess, [&](size_t i, size_t j, Real v) {
    //   CHECK(doctest::Approx(gt_hess.coeff(i, j)) == v);
    // });
  }
}

#ifdef AX_HAS_CUDA
TEST_CASE("Elast GPU") {
  auto state = std::make_shared<State>(3, 8, BufferDevice::Device);
  auto mesh = create_cube(BufferDevice::Device);
  ElasticityTerm term(state, mesh);
  auto lame = term.compute_.Lame()->View();
  math::RealField2 u_lame(2, 5);
  u_lame.colwise() = elasticity::compute_lame(1e7, 0.3);
  copy(lame, view_from_matrix(u_lame));

  state->GetVariables()->SetBytes(0);

  state->GetVariables()->SetBytes(0);
  SUBCASE("Zero") {
    term.UpdateGradient();
    term.UpdateEnergy();

    auto e = term.GetEnergy();
    CHECK(doctest::Approx(e) == 0);
  }
}
#endif