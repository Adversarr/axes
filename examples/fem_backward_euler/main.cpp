#include <imgui.h>

#include "ax/core/buffer/copy.hpp"
#include "ax/core/buffer/create_buffer.hpp"
#include "ax/core/buffer/eigen_support.hpp"
#include "ax/core/entt.hpp"
#include "ax/core/init.hpp"
#include "ax/fem/elasticity/base.hpp"
#include "ax/fem/timestep2/timestep.hpp"
#include "ax/geometry/primitives.hpp"
#include "ax/gl/context.hpp"
#include "ax/gl/init.hpp"
#include "ax/gl/primitives/mesh.hpp"
#include "ax/math/accessor.hpp"
#include "ax/math/buffer_blas.hpp"
#include "ax/math/views.hpp"

using namespace ax;

Entity entity;

struct FemData {
  std::shared_ptr<fem::Mesh> mesh_;
  std::shared_ptr<fem::TimeStepBase> timestep_;

  explicit FemData(std::shared_ptr<fem::Mesh> mesh)
      : mesh_(mesh), timestep_(std::make_shared<fem::TimeStepBase>(mesh)) {}
};

void update_render() {
  auto& data = get_component<FemData>(entity);
  auto& mesh = data.mesh_;
  auto vertices = mesh->GetVertices();
  auto u_dev = data.timestep_->GetProblem().GetState()->GetVariables()->ConstView();
  auto u_host = create_buffer<Real>(BufferDevice::Host, u_dev.Shape());
  auto u = u_host->View();
  copy(u, u_dev);

  patch_component<gl::Mesh>(entity, [&](gl::Mesh& m) {
    auto view = view_from_matrix(m.vertices_);
    copy(view, vertices->View());
    math::buffer_blas::axpy(1.0, u, view);
    m.colors_.topRows<3>() = m.vertices_.cwiseAbs();
    m.use_lighting_ = false;
  });
}

void on_frame(gl::UiRenderEvent) {
  auto& data = get_component<FemData>(entity);
  bool need_update = false;
  if (ImGui::Begin("Finite Element")) {
    if (ImGui::Button("Step")) {
      need_update = true;
    }
    static bool running = false;
    ImGui::Checkbox("Running", &running);
    need_update |= running;
  }
  ImGui::End();
  if (need_update) {
    data.timestep_->Step();
    update_render();
  }
}

int main(int argc, char* argv[]) {
  po::add_option({
    po::make_option<bool>("gpu", "Use gpu compute", "true"),
    po::make_option<int>("size", "Size of the mesh", "4"),
  });

  gl::init(argc, argv);
  entity = create_entity();

  auto use_gpu = po::get<bool>("gpu");
  auto size = po::get<int>("size");

  BufferDevice device = use_gpu ? BufferDevice::Device : BufferDevice::Host;

  auto cube = geo::tet_cube(1, size, size, size);
  auto mesh = std::make_shared<fem::Mesh>(3, 4, device);

  auto vertices = cube.vertices_;
  auto elements = cube.indices_.cast<size_t>().eval();
  mesh->SetData(view_from_matrix(vertices), view_from_matrix(elements));

  size_t n_elem = mesh->GetNumElements();
  size_t n_triangles = n_elem * 4;
  size_t n_vertices = mesh->GetNumVertices();
  auto& fe = add_component<FemData>(entity, mesh);
  auto state = fe.timestep_->GetProblem().GetState();

  std::vector<fem::VariableCondition> conditions(3 * n_vertices, fem::VariableCondition::None);
  for (size_t i = 0; i < static_cast<size_t>(size * 3); ++i) {
    conditions[i] = fem::VariableCondition::Dirichlet;
  }
  state->SetCondition(view_from_buffer(conditions, {3, n_vertices}));
  fe.timestep_->UpdatePruneDirichletBc();

  auto& glmesh = add_component<gl::Mesh>(entity);

  glmesh.indices_.resize(3, static_cast<Index>(n_triangles));
  for (auto [tid, tet] : math::enumerate(math::make_accessor(elements))) {
    Index i = static_cast<Index>(tet(0)), j = static_cast<Index>(tet(1)),
          k = static_cast<Index>(tet(2)), l = static_cast<Index>(tet(3));
    glmesh.indices_.col(tid * 4 + 0) = math::IndexVector3(i, j, k);
    glmesh.indices_.col(tid * 4 + 1) = math::IndexVector3(j, k, l);
    glmesh.indices_.col(tid * 4 + 2) = math::IndexVector3(k, l, i);
    glmesh.indices_.col(tid * 4 + 3) = math::IndexVector3(l, i, j);
  }
  glmesh.colors_.resize(4, static_cast<Index>(n_vertices));
  glmesh.vertices_ = vertices;

  auto lame = fem::elasticity::compute_lame(3e6, 0.33);
  fe.timestep_->SetElasticity(fem::ElasticityKind::StableNeoHookean);
  fe.timestep_->SetExternalAcceleration(math::RealVector3{0, -9.8, 0});
  fe.timestep_->SetDensity(1e3);
  fe.timestep_->SetLame(lame);
  connect<gl::UiRenderEvent, &on_frame>();
  get_resource<gl::Context>().Initialize();
  return gl::enter_main_loop();
}