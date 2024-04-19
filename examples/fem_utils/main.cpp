#include <imgui.h>

#include "ax/core/entt.hpp"
#include "ax/core/init.hpp"
#include "ax/fem/mass_matrix.hpp"
#include "ax/gl/utils.hpp"
#include "ax/graph/node.hpp"
#include "ax/graph/render.hpp"
#include "ax/nodes/geometry.hpp"
#include "ax/nodes/gl_prims.hpp"
#include "ax/nodes/io.hpp"
#include "ax/nodes/math_types.hpp"
#include "ax/nodes/stl_types.hpp"
#include "ax/utils/status.hpp"

using namespace ax;
using namespace ax::graph;
using namespace ax::fem;
constexpr idx dim = 3;

class Make_P1Mesh3D : public NodeBase {
public:
  Make_P1Mesh3D(NodeDescriptor const *descriptor, idx id) : NodeBase(descriptor, id) {}

  static void register_this() {
    NodeDescriptorFactory<Make_P1Mesh3D>{}
        .SetName("Make_P1Mesh3D")
        .SetDescription("Create a 3D FEM mesh.")
        .AddInput<Entity>("entity", "The entity to attach.")
        .AddInput<math::field3r>("vertices", "The vertices of the mesh.")
        .AddInput<math::field4i>("tetras", "The elements of the mesh.")
        .AddInput<bool>("reload", "Reload every frame")
        .AddOutput<Entity>("entity", "The entity attached with mesh.")
        .FinalizeAndRegister();
  }

  Status DoApply() {
    auto *entity = RetriveInput<Entity>(0);
    auto *vertices = RetriveInput<math::field3r>(1);
    auto *tetras = RetriveInput<math::field4i>(2);

    if (entity == nullptr || *entity == entt::null) {
      return utils::FailedPreconditionError("Missing input entity");
    } else if (vertices == nullptr) {
      return utils::FailedPreconditionError("Missing input vertices");
    } else if (tetras == nullptr) {
      return utils::FailedPreconditionError("Missing input tetras");
    }
    auto mesh = MeshBase<3>::Create(MeshType::kP1);
    if (! mesh) {
      return utils::FailedPreconditionError("Failed to create mesh.");
    }

    auto s = mesh->SetMesh(*tetras, *vertices);
    if (!s.ok()) {
      return s;
    }

    add_or_replace_component<UPtr<MeshBase<3>>>(*entity, std::move(mesh));
    AX_RETURN_OK();
  }

  Status Apply(idx fid) final {
    if (auto reload = RetriveInput<bool>(1); fid == 0 || (reload != nullptr && *reload)) {
      if (auto e = RetriveInput<Entity>(0); e != nullptr) {
        SetOutput<Entity>(0, *e);
      }
      return DoApply();
    }
    AX_RETURN_OK();
  }

  Status PreApply() final {
    if (auto e = RetriveInput<Entity>(0); e != nullptr) {
      SetOutput<Entity>(0, *e);
    }
    DoApply().IgnoreError();
    AX_RETURN_OK();
  }
};

class ComputeMassMatrix : public NodeBase {
public:
  ComputeMassMatrix(NodeDescriptor const *descriptor, idx id) : NodeBase(descriptor, id) {}

  static void register_this() {
    NodeDescriptorFactory<ComputeMassMatrix>{}
        .SetName("ComputeMassMatrix")
        .SetDescription("Compute the mass matrix.")
        .AddInput<Entity>("entity", "The entity attached with mesh.")
        .AddInput<real>("u_density", "Density for your object, default=1e1")
        .AddInput<bool>("lumped",
                        "Compute the lumped mass instead of standard FEM one. default=false")
        .AddInput<bool>("reload", "Reload every frame.")
        .AddOutput<math::sp_matxxr>("mass_matrix", "The mass matrix.")
        .FinalizeAndRegister();
  }

  Status DoApply() {
    auto *entity = RetriveInput<Entity>(0);
    if (entity == nullptr) {
      return utils::FailedPreconditionError("Missing input entity");
    }

    auto *mesh = try_get_component<UPtr<MeshBase<3>>>(*entity);
    if (mesh == nullptr) {
      return utils::FailedPreconditionError("Entity does not have a mesh.");
    }

    auto in_u_density = RetriveInput<real>(1);
    real u_density = in_u_density == nullptr ? real(1.0e1) : real(*in_u_density);
    if (u_density <= 0) {
      return utils::FailedPreconditionError("Density must be positive.");
    }

    MassMatrixCompute<3> mmc(**mesh);
    idx dofs = mesh->get()->GetNumVertices() * 3;
    if (auto lumped = RetriveInput<bool>(2); true || lumped == nullptr || !(*lumped)) {
      auto mass_matrix = mmc(u_density);
      SetOutput<math::sp_matxxr>(0, math::make_sparse_matrix(dofs, dofs, mass_matrix));
    }
    AX_RETURN_OK();
  }

  Status Apply(idx fid) final {
    if (auto reload = RetriveInput<bool>(1); fid == 0 || (reload != nullptr && *reload)) {
      return DoApply();
    }
    AX_RETURN_OK();
  }

  Status PreApply() final {
    DoApply().IgnoreError();
    AX_RETURN_OK();
  }
};

UPtr<MeshBase<dim>> mesh_;
UPtr<MassMatrixCompute<dim>> mm_compute_;

int main(int argc, char **argv) {
  gl::init(argc, argv);
  graph::install_renderer();
  nodes::register_stl_types();
  nodes::register_io_nodes();
  nodes::register_math_types_nodes();
  nodes::register_gl_prim_nodes();
  nodes::register_geometry_nodes();

  Make_P1Mesh3D::register_this();
  ComputeMassMatrix::register_this();

  AX_CHECK_OK(gl::enter_main_loop());
  clean_up();
  return 0;
}
