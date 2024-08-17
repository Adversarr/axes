#include <imgui.h>
#include "ax/core/init.hpp"
#include "ax/geometry/common.hpp"
#include "ax/gl/utils.hpp"
#include "ax/graph/render.hpp"
#include "ax/nodes/geometry.hpp"
#include "ax/nodes/gl_prims.hpp"
#include "ax/nodes/io.hpp"
#include "ax/nodes/math_types.hpp"
#include "ax/nodes/stl_types.hpp"
#include "ax/utils/status.hpp"
#include "solver.hpp"

using namespace ax;
using namespace ax::graph;

class DijkstraPath : public NodeBase {
  public:
  DijkstraPath(NodeDescriptor const* descriptor, Index id) : NodeBase(descriptor, id) { }

  static void register_this() {
    NodeDescriptorFactory<DijkstraPath>()
      .SetName("Compute_dijkstra_path")
      .SetDescription("Compute geodesic path between two points on a triangle mesh")
      .AddInput<geo::SurfaceMesh>("mesh", "Triangle mesh")
      .AddInput<Index>("start", "Start vertex index")
      .AddInput<Index>("end", "End vertex index")
      .AddOutput<Path>("path", "Geodesic path between two points")
      .FinalizeAndRegister();
  }

  Status Apply(Index) final {
    auto mesh = RetriveInput<geo::SurfaceMesh>(0);
    if (!mesh) {
      return utils::FailedPreconditionError("Input mesh is not connected");
    }

    Index start = 0;
    Index end = 1;
    if (auto start_input = RetriveInput<Index>(1)) {
      start = *start_input;
    }
    if (auto end_input = RetriveInput<Index>(2)) {
      end = *end_input;
    }

    auto solver = Dijkstra(*mesh);
    auto path = solver.ShortestPath(start, end);

    SetOutput<Path>(0, path);
    AX_RETURN_OK();
  }
};

class PathToLines : public NodeBase {
public:
  PathToLines(NodeDescriptor const* descriptor, Index id) : NodeBase(descriptor, id) { }

  static void register_this() {
    NodeDescriptorFactory<PathToLines>()
      .SetName("Path_to_lines")
      .SetDescription("Convert geodesic path to lines")
      .AddInput<Path>("path", "Geodesic path between two points")
      .AddInput<math::RealField3>("vertices", "Trimesh")
      .AddInput<math::IndexField3>("faces", "Trimesh")
      .AddOutput<math::RealField3>("vertices", "")
      .FinalizeAndRegister();
  }

  Status Apply(Index) final {
    auto path = RetriveInput<Path>(0);
    auto vertices_in = RetriveInput<math::RealField3>(1);
    auto faces = RetriveInput<math::IndexField3>(2);
    if (!path) {
      return utils::FailedPreconditionError("Input path is not connected");
    } else if (!vertices_in) {
      return utils::FailedPreconditionError("Input vertices is not connected");
    } else if (!faces) {
      return utils::FailedPreconditionError("Input faces is not connected");
    }

    math::RealField3 vertices(3, path->size());
    if (path->empty()) {
      SetOutput<math::RealField3>(0, math::RealVector3::Zero(3, 0));
      std::cout << "empty path" << std::endl;
      AX_RETURN_OK();
    }
    for (size_t i = 0; i < path->size(); ++i) {
      auto p = (*path)[i];
      if (p.on_edge_) {
        Index fid = p.id_;
        Index eid = p.which_;
        Index I = (*faces)(eid, fid);
        Index J = (*faces)((eid + 1) % 3, fid);
        auto p0 = (*vertices_in).col(I);
        auto p1 = (*vertices_in).col(J);
        vertices.col(i) = p.rel_pos_ * p1 + (1 - p.rel_pos_) * p0;
      } else {
        vertices.col(i) = (*vertices_in).col(p.id_);
      }
    }
    std::cout << vertices << std::endl;
    SetOutput<math::RealField3>(0, vertices);
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

  DijkstraPath::register_this();
  PathToLines::register_this();

  AX_CHECK_OK(gl::enter_main_loop());
  clean_up();
  return 0;
}
