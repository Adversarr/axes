#include <imgui.h>
#include "ax/core/init.hpp"
#include "ax/gl/utils.hpp"
#include "ax/graph/render.hpp"
#include "ax/nodes/geometry.hpp"
#include "ax/nodes/gl_prims.hpp"
#include "ax/nodes/io.hpp"
#include "ax/nodes/math_types.hpp"
#include "ax/nodes/stl_types.hpp"
#include "ax/utils/status.hpp"

#include "stylization.hpp"

using namespace ax;
using namespace ax::graph;

class CubicStylization : public NodeBase {
public:
  CubicStylization(NodeDescriptor const* descriptor, Index id) : NodeBase(descriptor, id) {}

  static void register_this() {
    NodeDescriptorFactory<CubicStylization>()
        .SetName("CubicStylization")
        .SetDescription("Stylizes a mesh using cubic splines")
        .AddInput<geo::SurfaceMesh>("mesh", "The mesh to stylize")
        .AddInput<Index>("steps", "The number of steps to take")
        .AddInput<Real>("rho", "rho")
        .AddInput<Real>("lambda", "lambda")
        .AddInput<Real>("tau", "tau")
        .AddInput<Real>("mu", "mu")
        .AddOutput<geo::SurfaceMesh>("mesh", "The stylized mesh")
        .FinalizeAndRegister();
  }

  Status Apply(Index f) override {
    auto *mesh = RetriveInput<geo::SurfaceMesh>(0);
    auto *steps = RetriveInput<Index>(1);
    auto *rho = RetriveInput<Real>(2);
    auto *lambda = RetriveInput<Real>(3);
    auto *tau = RetriveInput<Real>(4);
    auto *mu = RetriveInput<Real>(5);

    if (mesh == nullptr) {
      return utils::FailedPreconditionError("Input mesh is null");
    }

    Dijkstra solver(*mesh);
    if (rho != nullptr) {
      solver.rho_ = *rho;
    }
    if (lambda != nullptr) {
      solver.lambda_ = *lambda;
    }

    if (tau != nullptr) {
      solver.tau_ = *tau;
    }

    if (mu != nullptr) {
      solver.mu_ = *mu;
    }
    solver.Step(steps == nullptr ? 1 : *steps);
    SetOutput<geo::SurfaceMesh>(0, solver.GetResult());
    AX_RETURN_OK();
  }
};



int main(int argc, char** argv) {
  gl::init(argc, argv);
  graph::install_renderer();
  nodes::register_stl_types();
  nodes::register_io_nodes();
  nodes::register_math_types_nodes();
  nodes::register_gl_prim_nodes();
  nodes::register_geometry_nodes();

  CubicStylization::register_this();

  AX_CHECK_OK(gl::enter_main_loop());
  clean_up();
  return 0;
}
