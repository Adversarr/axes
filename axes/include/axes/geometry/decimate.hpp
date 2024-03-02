#pragma once

#include "axes/geometry/halfedge.hpp"
namespace ax::geo {

struct EdgeCollapseCost {
  HalfedgeEdge_t* edge;
  math::vec3r target_position;
  real cost;
  bool operator<(EdgeCollapseCost const& other) const {
    return cost > other.cost;
  }
};

class MeshDecimator {
public:
  MeshDecimator(HalfedgeMesh* mesh_);

  MeshDecimator& SetRatio(real ratio);

  MeshDecimator& SetTargetCount(idx count);

  Status Run();

private:

  HalfedgeEdge_t* FindEdgeToCollapse();

  real cost_threshold_{1e9};

  HalfedgeMesh* mesh_;

  idx target_count_;
};

}