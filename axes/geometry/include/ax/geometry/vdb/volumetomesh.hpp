#pragma once

#include "ax/core/common.hpp"
#include "ax/geometry/common.hpp"
#include "ax/vdb/common.hpp"


namespace ax::vdb {

class VolumeToMesh {
public:
  VolumeToMesh(Real isovalue = 0, Real adaptivity = 0, bool relaxDisorientedTriangles = true);

  ~VolumeToMesh();

  geo::SurfaceMesh operator()(vdb::RealGridPtr tree) const;

  struct Impl;
private:
  std::unique_ptr<Impl> impl_;
};

}