#pragma once
#include "ax/geometry/common.hpp"

using namespace ax;
using namespace ax::geo;

class Solver {
public:
  Solver(SurfaceMesh mesh) : mesh_(mesh) {}

  void Step(idx steps);

  SurfaceMesh const & GetResult() const { return mesh_; }

private:
  SurfaceMesh mesh_;
};