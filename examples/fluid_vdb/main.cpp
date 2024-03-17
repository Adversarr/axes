#include <absl/flags/declare.h>
#include <absl/flags/flag.h>
#include <igl/readOBJ.h>
#include <imgui.h>
#include <openvdb/points/PointScatter.h>
#include <openvdb/tools/LevelSetSphere.h>

#include <axes/gl/utils.hpp>

#include "axes/components/name.hpp"
#include "axes/core/echo.hpp"
#include "axes/core/entt.hpp"
#include "axes/core/init.hpp"
#include "axes/geometry/io.hpp"
#include "axes/geometry/normal.hpp"
#include "axes/geometry/sample/mesh2pc.hpp"
#include "axes/gl/context.hpp"
#include "axes/gl/primitives/lines.hpp"
#include "axes/gl/primitives/mesh.hpp"
#include "axes/gl/primitives/points.hpp"
#include "axes/gl/primitives/quiver.hpp"
#include "axes/utils/asset.hpp"
#include "axes/vdb/pointcloud.hpp"
#include "axes/vdb/volumetomesh.hpp"

using namespace ax;

int main(int argc, char** argv) {
  ax::gl::init(argc, argv);
  {
    vdb::PointDataGrid::Ptr points;
    vdb::RealGridPtr v_current, v_next, v_diff;
    vdb::RealGridPtr pressure;
    vdb::RealGridPtr fluid_ls_init
        = openvdb::tools::createLevelSetSphere<vdb::RealGrid>(0.5, vdb::Vec3r(0, 0, 1), 0.04);
    points = openvdb::points::denseUniformPointScatter(*fluid_ls_init, 8);
    points->setName("points");
    openvdb::points::appendAttribute<vdb::Vec3r>(points->tree(), "velocity", vdb::Vec3r(0, 0, -1),
                                                 1, true, nullptr, false, false);

    
  }
  AX_CHECK_OK(gl::enter_main_loop());
  ax::clean_up();
  return 0;
}
