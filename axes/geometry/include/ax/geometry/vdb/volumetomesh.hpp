#pragma once

#include "ax/geometry/common.hpp"
#include "ax/utils/opt.hpp"
#include "ax/vdb/common.hpp"

namespace ax::vdb {
class VolumeToMesh final : utils::Tunable {
public:
  explicit VolumeToMesh(Real isovalue = 0, Real adaptivity = 0,
                        bool relax_disoriented_triangles = true);

  ~VolumeToMesh() override;

  geo::SurfaceMesh operator()(vdb::RealGridPtr tree) const;

  void SetOptions(utils::Options const& option) override;

  utils::Options GetOptions() const override;

private:
  Real isovalue_ = 0;
  Real adaptivity_ = 0;
  bool relax_disoriented_triangles_ = true;
};
}