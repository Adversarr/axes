#include <imgui.h>

#include "ax/core/entt.hpp"
#include "ax/core/init.hpp"
#include "ax/fem/elasticity.hpp"
#include "ax/fem/elasticity/linear.hpp"
#include "ax/fem/elasticity/neohookean_bw.hpp"
#include "ax/fem/elasticity/stvk.hpp"
#include "ax/fem/trimesh.hpp"
#include "ax/geometry/io.hpp"
#include "ax/geometry/primitives.hpp"
#include "ax/gl/colormap.hpp"
#include "ax/gl/context.hpp"
#include "ax/gl/primitives/mesh.hpp"
#include "ax/gl/primitives/quiver.hpp"
#include "ax/gl/utils.hpp"
#include "ax/utils/asset.hpp"

ABSL_FLAG(bool, flip_yz, false, "flip yz");

using namespace ax;
Entity out;
geo::TetraMesh input_mesh;
math::FloatVector3 stretching;
math::RealVector2 lame;

void update_entity() {
  gl::Mesh& msh = add_or_replace_component<gl::Mesh>(out);
  msh.vertices_ = input_mesh.vertices_;
  msh.indices_.resize(3, input_mesh.indices_.cols() * 4);
  for (Index i = 0; i < input_mesh.indices_.cols(); ++i) {
    msh.indices_.col(i * 4 + 0) = math::IndexVector3{input_mesh.indices_(0, i), input_mesh.indices_(1, i),
                                              input_mesh.indices_(2, i)};
    msh.indices_.col(i * 4 + 1) = math::IndexVector3{input_mesh.indices_(0, i), input_mesh.indices_(1, i),
                                              input_mesh.indices_(3, i)};
    msh.indices_.col(i * 4 + 2) = math::IndexVector3{input_mesh.indices_(0, i), input_mesh.indices_(2, i),
                                              input_mesh.indices_(3, i)};
    msh.indices_.col(i * 4 + 3) = math::IndexVector3{input_mesh.indices_(1, i), input_mesh.indices_(2, i),
                                              input_mesh.indices_(3, i)};
  }
  for (auto v : math::each(msh.vertices_)) {
    v = (v.array() * stretching.cast<Real>().array());
  }
  auto V = msh.vertices_;
  auto F = input_mesh.indices_;
  msh.colors_ = math::ones<4>(V.cols()) * 0.5;
  msh.is_flat_ = false;
  msh;
  std::shared_ptr<fem::LinearMesh<3>> mesh = std::make_shared<fem::LinearMesh<3>>();
  mesh->SetMesh(F, V);
  fem::ElasticityCompute_CPU<3, fem::elasticity::NeoHookeanBW> elast(mesh);
  elast.SetLame(lame);
  // add_or_replace_component<gl::Lines>(out, gl::Lines::Create(msh));
  elast.Update(mesh->GetVertices(), ax::fem::ElasticityUpdateLevel::Hessian);
  elast.UpdateStress();
  elast.GatherStressToVertices();
  auto force = elast.GetStressOnVertices();
  auto& q = add_or_replace_component<gl::Quiver>(out);
  q.colors_.setOnes(4, V.cols());
  q.positions_ = V;
  q.directions_ = force;
  elast.UpdateEnergy();
  elast.GatherEnergyToVertices();
  auto energy = elast.GetEnergyOnVertices();
  Real m = energy.minCoeff(), M = energy.maxCoeff();
  gl::Colormap mapping(m, M);
  AX_LOG(INFO) << "m=" << m << "\tM=" << M;
  auto result = mapping(energy.transpose());
  msh.colors_.topRows<3>() = result;
  q.scale_ = 0.15;
  // q.normalize_ = true;
  q;
}

static void ui_callback(gl::UiRenderEvent const&) {
  ImGui::Begin("FEM Implicit");
  bool has_shearing_changed = false;
  has_shearing_changed |= ImGui::SliderFloat("Scale X", &stretching.x(), 0.0f, 2.0f);
  has_shearing_changed |= ImGui::SliderFloat("Scale Y", &stretching.y(), 0.0f, 2.0f);
  has_shearing_changed |= ImGui::SliderFloat("Scale Z", &stretching.z(), 0.0f, 2.0f);
  if (has_shearing_changed) {
    update_entity();
  }
  ImGui::End();
}

int main(int argc, char** argv) {
  ax::gl::init(argc, argv);
  lame = fem::elasticity::compute_lame(1e4, 0.45);
  input_mesh = geo::tet_cube(0.5, 10, 10, 10);

  // auto ele = geo::read_ele(utils::get_asset("/mesh/tet/house-ele-node/house.ele"));
  // auto node = geo::read_node(utils::get_asset("/mesh/tet/house-ele-node/house.node"));
  // if (!ele.ok() || !node.ok()) {
  //   AX_LOG(WARNING) << "Failed to read mesh.";
  // }
  // input_mesh.vertices_ = node.value().vertices_;
  // input_mesh.indices_ = ele.value().elements_;

  AX_LOG(INFO) << "#V=" << input_mesh.vertices_.cols() << "\t#E=" << input_mesh.indices_.cols();

  stretching.setOnes();
  update_entity();
  connect<gl::UiRenderEvent, &ui_callback>();
  AX_CHECK_OK(gl::enter_main_loop());
  clean_up();
  return 0;
}
