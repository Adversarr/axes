#pragma once

#include "ax/geometry/common.hpp"
#include <map>


using namespace ax;

// You Can Find Geodesic Paths in Triangle Meshes by Just Flipping Edges

struct PathItem {
  Index id_;
  bool on_edge_;
  real rel_pos_;
  Index which_;
};

using Path = std::vector<PathItem>;

class Dijkstra {
public:
  Dijkstra(geo::SurfaceMesh mesh);

  Path ShortestPath(Index start, Index end);

private:
  geo::SurfaceMesh mesh_;

  std::vector<std::vector<Index>> adj_list_;
};


Path make_shortest_geodesic(geo::SurfaceMesh mesh, Path p0);