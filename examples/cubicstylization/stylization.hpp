#pragma once
#include "ax/geometry/common.hpp"

using namespace ax;
using namespace ax::geo;

class Dijkstra {
public:
  Dijkstra(SurfaceMesh mesh) : mesh_(mesh) {}

  void Step(Index steps);

  SurfaceMesh const & GetResult() const { return mesh_; }

  std::vector<math::RealField3> cached_sequence;

  SurfaceMesh mesh_;

  Real rho_ = 1e-3;
  Real lambda_ = 1.0;
  Real tau_ = 2.0;
  Real mu_ = 10.0;
};