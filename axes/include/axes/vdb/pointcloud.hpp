#pragma once

#include "common.hpp"
#include <openvdb/points/PointDataGrid.h>
#include <openvdb/tools/PointIndexGrid.h>
#include <openvdb/points/PointConversion.h>
#include <openvdb/points/PointCount.h>
namespace ax::vdb {

using openvdb::points::PointDataGrid;
using openvdb::tools::PointIndexGrid;

class PointGrid {
public:
  PointGrid(math::field3r const& position, real voxel_size=-1, idx point_per_voxel=8);

  PointDataGrid::Ptr DataGrid() { return point_data_grid_; }

  PointIndexGrid::Ptr IndexGrid() { return point_index_grid_; }

  openvdb::math::Transform::Ptr Transform() { return transform_; }

  Vec3rGridPtr TransferStaggered(std::string const& name, math::field3r const& field);

  Vec3rGridPtr TransferCellCenter(std::string const& name, math::field3r const& field);

private:
  PointDataGrid::Ptr point_data_grid_;
  PointIndexGrid::Ptr point_index_grid_;
  openvdb::math::Transform::Ptr transform_;
};

}