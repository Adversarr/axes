#include "ax/vdb/pointcloud.hpp"
#include "ax/core/logging.hpp"
#include <openvdb/points/PointRasterizeTrilinear.h>

namespace ax::vdb {

PointGrid::PointGrid(math::field3r const& position, real voxel_size, idx point_per_voxel) {
  std::vector<openvdb::Vec3R> positions;
  positions.reserve(static_cast<size_t>(position.cols()));
  for (auto p : math::each(position)) {
    positions.push_back(openvdb::Vec3R(p.x(), p.y(), p.z()));
  }
  AX_CHECK(point_per_voxel > 0, "points_per_voxel must be greater than 0");

  openvdb::points::PointAttributeVector<openvdb::Vec3R> position_wrapper(positions);
  if (voxel_size <= 0) {
    voxel_size = openvdb::points::computeVoxelSize(position_wrapper, static_cast<uint32_t>(point_per_voxel));
    AX_INFO("Voxel Size: {}", voxel_size);
  }

  transform_ = openvdb::math::Transform::createLinearTransform(voxel_size);

  point_index_grid_ = openvdb::tools::createPointIndexGrid<openvdb::tools::PointIndexGrid>(
      position_wrapper, *transform_);
  point_data_grid_ = openvdb::points::createPointDataGrid<openvdb::points::NullCodec,
                                                          openvdb::points::PointDataGrid>(
      *point_index_grid_, position_wrapper, *transform_);

  point_data_grid_->setName("Points");
}

Vec3rGridPtr PointGrid::TransferStaggered(std::string const& name, math::field3r const& field) {
  size_t cnt = static_cast<size_t>(field.cols());
  std::vector<openvdb::Vec3R> values;
  values.reserve(static_cast<size_t>(cnt));
  for (auto v : math::each(field)) {
    values.push_back(openvdb::Vec3R(v.x(), v.y(), v.z()));
  }
  openvdb::points::PointAttributeVector<openvdb::Vec3R> value_wrapper(values);

  using Codec = openvdb::points::NullCodec;
  openvdb::points::TypedAttributeArray<vdb::Vec3r, Codec>::registerType();
  openvdb::NamePair transfer_attribute
      = openvdb::points::TypedAttributeArray<vdb::Vec3r, Codec>::attributeType();
  openvdb::points::appendAttribute(point_data_grid_->tree(), name, transfer_attribute);
  openvdb::points::populateAttribute(point_data_grid_->tree(), point_index_grid_->tree(), name,
                                     value_wrapper);

  auto transferred_tree = openvdb::DynamicPtrCast<vdb::Vec3rTree>(
      openvdb::points::rasterizeTrilinear<true, vdb::Vec3r>(point_data_grid_->tree(), name));
  auto transferred_grid = vdb::Vec3rGrid::create(transferred_tree)->deepCopy();
  transferred_grid->setTransform(transform_);
  return transferred_grid;
}

Vec3rGridPtr PointGrid::TransferCellCenter(std::string const& name, math::field3r const& field) {
  size_t cnt = static_cast<size_t>(field.cols());
  std::vector<openvdb::Vec3R> values;
  values.reserve(cnt);
  for (auto v : math::each(field)) {
    values.push_back(openvdb::Vec3R(v.x(), v.y(), v.z()));
  }
  openvdb::points::PointAttributeVector<openvdb::Vec3R> value_wrapper(values);

  using Codec = openvdb::points::NullCodec;
  openvdb::points::TypedAttributeArray<vdb::Vec3r, Codec>::registerType();
  openvdb::NamePair transfer_attribute
      = openvdb::points::TypedAttributeArray<vdb::Vec3r, Codec>::attributeType();
  openvdb::points::appendAttribute(point_data_grid_->tree(), name, transfer_attribute);
  openvdb::points::populateAttribute(point_data_grid_->tree(), point_index_grid_->tree(), name,
                                     value_wrapper);

  auto transferred_tree = openvdb::DynamicPtrCast<vdb::Vec3rTree>(
      openvdb::points::rasterizeTrilinear<false, vdb::Vec3r>(point_data_grid_->tree(), name));
  auto transferred_grid = vdb::Vec3rGrid::create(transferred_tree)->deepCopy();
  transferred_grid->setTransform(transform_);
  return transferred_grid;
}


RealGridPtr PointGrid::TransferCellCenter(std::string const& name, math::field1r const& field) {
  size_t cnt = static_cast<size_t>(field.cols());
  std::vector<real> values;
  values.reserve(cnt);
  for (auto v : math::each(field)) {
    values.push_back(v.x());
  }
  openvdb::points::PointAttributeVector<real> value_wrapper(values);

  using Codec = openvdb::points::NullCodec;
  openvdb::points::TypedAttributeArray<real, Codec>::registerType();
  openvdb::NamePair transfer_attribute = openvdb::points::TypedAttributeArray<real, Codec>::attributeType();
  openvdb::points::appendAttribute(point_data_grid_->tree(), name, transfer_attribute);
  openvdb::points::populateAttribute(point_data_grid_->tree(), point_index_grid_->tree(), name,
                                     value_wrapper);

  auto transferred_tree = openvdb::DynamicPtrCast<vdb::RealTree>(
      openvdb::points::rasterizeTrilinear<false, real>(point_data_grid_->tree(), name));
  auto transferred_grid = vdb::RealGrid::create(transferred_tree)->deepCopy();
  transferred_grid->setTransform(transform_);
  return transferred_grid;
}

}  // namespace ax::vdb
