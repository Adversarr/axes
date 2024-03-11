#pragma once

#include "axes/core/common.hpp"
#include "axes/geometry/common.hpp"
#include "axes/vdb/common.hpp"


namespace ax::vdb {

class VolumeToMesh {
public:
  VolumeToMesh(real isovalue = 0, real adaptivity = 0, bool relaxDisorientedTriangles = true);

  ~VolumeToMesh();

  geo::SurfaceMesh operator()(vdb::RealGridPtr tree) const;

  struct Impl;
private:
  std::unique_ptr<Impl> impl_;
};

}