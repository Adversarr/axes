#pragma once

#include "ax/geometry/common.hpp"
#include <map>


using namespace ax;

// You Can Find Geodesic Paths in Triangle Meshes by Just Flipping Edges

struct PathItem {
  idx id_;
  bool on_edge_;
  real rel_pos_;
  idx which_;
};

using Path = List<PathItem>;

class Dijkstra {
public:
  Dijkstra(geo::SurfaceMesh mesh);

  Path ShortestPath(idx start, idx end);

private:
  geo::SurfaceMesh mesh_;

  List<List<idx>> adj_list_;
};


Path make_shortest_geodesic(geo::SurfaceMesh mesh, Path p0);