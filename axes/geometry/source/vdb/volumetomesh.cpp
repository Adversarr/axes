#include "ax/geometry/vdb/volumetomesh.hpp"

#include <openvdb/tools/VolumeToMesh.h>

namespace ax::vdb {

struct VolumeToMesh::Impl {
  openvdb::tools::VolumeToMesh algo_;
  Impl(Real isovalue, Real adaptivity, bool relaxDisorientedTriangles)
      : algo_(isovalue, adaptivity, relaxDisorientedTriangles) {};
};

VolumeToMesh::VolumeToMesh(Real isovalue, Real adaptivity, bool relaxDisorientedTriangles) {
  impl_ = std::make_unique<Impl>(isovalue, adaptivity, relaxDisorientedTriangles);
};

VolumeToMesh::~VolumeToMesh() = default;

geo::SurfaceMesh VolumeToMesh::operator()(vdb::RealGridPtr tree) const {
  std::vector<openvdb::Vec3s> points;
  std::vector<openvdb::Vec3I> triangles;
  std::vector<openvdb::Vec4I> quads;
  auto& mesher = impl_->algo_;

  mesher(*tree);

  for (size_t i = 0; i < mesher.pointListSize(); ++i) {
    points.push_back(mesher.pointList()[i]);
  }

  for (size_t i = 0; i < mesher.polygonPoolListSize(); ++i) {
    auto& polygon_list = mesher.polygonPoolList()[i];
    for (size_t j = 0; j < polygon_list.numTriangles(); ++j) {
      triangles.push_back(polygon_list.triangle(j));
    }
    for (size_t j = 0; j < polygon_list.numQuads(); ++j) {
      openvdb::Vec4I quad = polygon_list.quad(j);
      triangles.push_back(openvdb::Vec3I(quad[0], quad[1], quad[2]));
      triangles.push_back(openvdb::Vec3I(quad[0], quad[2], quad[3]));
    }
  }

  math::RealField3 vertices(3, points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    vertices.col(static_cast<Index>(i)) = math::RealVector3(points[i].x(), points[i].y(), points[i].z());
  }

  math::IndexField3 indices(3, triangles.size());
  for (size_t i = 0; i < triangles.size(); ++i) {
    indices.col(static_cast<Index>(i))
        = math::IndexVector3(triangles[i].x(), triangles[i].y(), triangles[i].z());
  }

  return {vertices, indices};
}

}  // namespace ax::vdb
