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
constexpr int dim = 3;

class Make_P1Mesh3D : public NodeBase {
public:
  Make_P1Mesh3D(NodeDescriptor const *descriptor, Index id) : NodeBase(descriptor, id) {}

  static void register_this() {
    NodeDescriptorFactory<Make_P1Mesh3D>{}
        .SetName("Make_P1Mesh3D")
        .SetDescription("Create a 3D FEM mesh.")
        .AddInput<Entity>("entity", "The entity to attach.")
        .AddInput<math::RealField3>("vertices", "The vertices of the mesh.")
        .AddInput<math::IndexField4>("tetras", "The elements of the mesh.")
        .AddInput<bool>("reload", "Reload every frame")
        .AddOutput<Entity>("entity", "The entity attached with mesh.")
        .FinalizeAndRegister();
  }

  Status DoApply() {
    auto *entity = RetriveInput<Entity>(0);
    auto *vertices = RetriveInput<math::RealField3>(1);
    auto *tetras = RetriveInput<math::IndexField4>(2);

    if (entity == nullptr || *entity == entt::null) {
      return utils::FailedPreconditionError("Missing input entity");
    } else if (vertices == nullptr) {
      return utils::FailedPreconditionError("Missing input vertices");
    } else if (tetras == nullptr) {
      return utils::FailedPreconditionError("Missing input tetras");
    }
    auto mesh = std::make_unique<TriMesh<3>>();

    mesh->SetMesh(*tetras, *vertices);
    add_or_replace_component<std::unique_ptr<TriMesh<3>>>(*entity, std::move(mesh));
    AX_RETURN_OK();
  }

  Status Apply(Index fid) final {
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
  ComputeMassMatrix(NodeDescriptor const *descriptor, Index id) : NodeBase(descriptor, id) {}

  static void register_this() {
    NodeDescriptorFactory<ComputeMassMatrix>{}
        .SetName("ComputeMassMatrix")
        .SetDescription("Compute the mass matrix.")
        .AddInput<Entity>("entity", "The entity attached with mesh.")
        .AddInput<Real>("u_density", "Density for your object, default=1e1")
        .AddInput<bool>("lumped",
                        "Compute the lumped mass instead of standard FEM one. default=false")
        .AddInput<bool>("reload", "Reload every frame.")
        .AddOutput<math::RealSparseMatrix>("mass_matrix", "The mass matrix.")
        .AddOutput<Real>("total", "total mass.")
        .FinalizeAndRegister();
  }

  Status DoApply() {
    auto *entity = RetriveInput<Entity>(0);
    if (entity == nullptr) {
      return utils::FailedPreconditionError("Missing input entity");
    }

    auto *mesh = try_get_component<std::unique_ptr<TriMesh<3>>>(*entity);
    if (mesh == nullptr) {
      return utils::FailedPreconditionError("Entity does not have a mesh.");
    }

    auto in_u_density = RetriveInput<Real>(1);
    Real u_density = in_u_density == nullptr ? Real(1.0e1) : Real(*in_u_density);
    if (u_density <= 0) {
      return utils::FailedPreconditionError("Density must be positive.");
    }

    MassMatrixCompute<3> mmc(**mesh);
    Index dofs = mesh->get()->GetNumVertices() * 3;
    auto mass_matrix = mmc(u_density);
    Real total = mass_matrix.sum();
    SetOutput<Real>(1, total / 3.0);
    AX_RETURN_OK();
  }

  Status Apply(Index fid) final {
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

std::unique_ptr<TriMesh<dim>> mesh_;
std::unique_ptr<MassMatrixCompute<dim>> mm_compute_;

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
