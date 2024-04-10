#include <imgui.h>

#include "ax/core/entt.hpp"
#include "ax/core/init.hpp"
#include "ax/geometry/io.hpp"
#include "ax/geometry/primitives.hpp"
#include "ax/gl/colormap.hpp"
#include "ax/gl/context.hpp"
#include "ax/gl/primitives/lines.hpp"
#include "ax/gl/primitives/mesh.hpp"
#include "ax/gl/primitives/quiver.hpp"
#include "ax/gl/utils.hpp"
#include "ax/fem/elasticity/stvk.hpp"
#include "ax/fem/elasticity/neohookean_bw.hpp"
#include "ax/fem/elasticity/linear.hpp"
#include "ax/fem/deform.hpp"
#include "ax/fem/elasticity.hpp"
#include "ax/fem/mesh/p1mesh.hpp"
#include "ax/utils/asset.hpp"

ABSL_FLAG(std::string, input, "square_naive.obj", "Input 2D Mesh.");
ABSL_FLAG(bool, flip_yz, false, "flip yz");

using namespace ax;
Entity out;
geo::SurfaceMesh input_mesh;
math::vec3f stretching;

math::vec2r lame;

void update_entity() {
  gl::Mesh& msh = add_or_replace_component<gl::Mesh>(out);
  msh.vertices_ = input_mesh.vertices_;
  for (auto v: math::each(msh.vertices_)) {
    v = (v.array() * stretching.cast<real>().array());
  }
  auto V = msh.vertices_;
  auto F = msh.indices_ = input_mesh.indices_;
  msh.colors_ = math::ones<4>(V.cols()) * 0.5;
  msh.is_flat_ = false;
  msh.flush_ = true;
  fem::P1Mesh<2> mesh;
  AX_CHECK_OK(mesh.SetMesh(F, V.topRows<2>()));
  fem::Deformation<2> deform(mesh, input_mesh.vertices_.topRows<2>());
  fem::ElasticityCompute<2, fem::elasticity::NeoHookeanBW> elast(deform);
  add_or_replace_component<gl::Lines>(out, gl::Lines::Create(msh)).flush_ = true;
  elast.UpdateDeformationGradient(mesh.GetVertices(), ax::fem::DeformationGradientUpdate::kHessian);
  auto stress = elast.Stress(lame);
  auto force = deform.StressToVertices(stress);
  auto& q = add_or_replace_component<gl::Quiver>(out);
  q.colors_.setOnes(4, V.cols());
  q.positions_ = V;
  q.directions_.resize(3, V.cols());
  q.directions_.topRows<2>() = force;
  q.directions_.row(2).setZero();
  auto energy = deform.EnergyToVertices(elast.Energy(lame));
  real m = energy.minCoeff(), M = energy.maxCoeff();
  gl::Colormap mapping(0, M);
  AX_LOG(INFO) << "m=" << m << "\tM=" << M;
  auto result = mapping(energy.transpose());
  msh.colors_.topRows<3>() = result;
  q.scale_ = 0.15;
  q.normalize_ = true;
  q.flush_ = true;
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
  input_mesh
      = ax::geo::read_obj(ax::utils::get_asset("/mesh/obj/" + absl::GetFlag(FLAGS_input))).value();
  if (absl::GetFlag(FLAGS_flip_yz)) {
    auto z = input_mesh.vertices_.row(2).eval();
    input_mesh.vertices_.row(2) = input_mesh.vertices_.row(1);
    input_mesh.vertices_.row(1) = z;
  }
  stretching.setOnes();
  update_entity();
  connect<gl::UiRenderEvent, &ui_callback>();
  AX_CHECK_OK(gl::enter_main_loop());
  clean_up();
  return 0;
}