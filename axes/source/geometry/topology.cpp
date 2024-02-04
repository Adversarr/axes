#include "axes/geometry/topology.hpp"

#include <absl/hash/hash.h>
#include <absl/log/check.h>

namespace ax::geo {

math::field2i get_edges(math::field3i const& triangles) {
  std::vector<utils::DupTuple<idx, 2>> edges;
  edges.reserve(triangles.cols() * 3);
  for (idx i = 0; i < triangles.cols(); ++i) {
    for (idx j = 0; j < 3; ++j) {
      idx a = triangles(j, i);
      idx b = triangles((j + 1) % 3, i);
      if (a > b) std::swap(a, b);
      edges.emplace_back(a, b);
    }
  }

  std::sort(edges.begin(), edges.end());
  auto end = std::unique_copy(edges.begin(), edges.end(), edges.begin());

  math::field2i edges_field(2, end - edges.begin());
  for (idx i = 0; i < edges_field.cols(); ++i) {
    edges_field(0, i) = std::get<0>(edges[i]);
    edges_field(1, i) = std::get<1>(edges[i]);
  }
  return edges_field;
}

math::field2i get_boundary_edges(math::field3i const& triangles) {
  std::vector<utils::DupTuple<idx, 2>> edges;

  for (idx i = 0; i < triangles.cols(); ++i) {
    for (idx j = 0; j < 3; ++j) {
      idx a = triangles(j, i);
      idx b = triangles((j + 1) % 3, i);
      if (a > b) std::swap(a, b);
      edges.emplace_back(a, b);
    }
  }

  std::sort(edges.begin(), edges.end());
  auto end = std::unique_copy(edges.begin(), edges.end(), edges.begin());

  math::field2i boundary_edges(2, end - edges.begin());
  for (idx i = 0; i < boundary_edges.cols(); ++i) {
    boundary_edges(0, i) = std::get<0>(edges[i]);
    boundary_edges(1, i) = std::get<1>(edges[i]);
  }
  return boundary_edges;
}

math::field3i get_boundary_triangles(math::field4i const& tetrahedrons) {
  std::vector<utils::DupTuple<idx, 3>> triangles;

  for (idx i = 0; i < tetrahedrons.cols(); ++i) {
    for (idx j = 0; j < 4; ++j) {
      idx a = tetrahedrons((j + 1) % 4, i);
      idx b = tetrahedrons((j + 2) % 4, i);
      idx c = tetrahedrons((j + 3) % 4, i);
      if (a > b) std::swap(a, b);
      if (a > c) std::swap(a, c);
      if (b > c) std::swap(b, c);
      triangles.push_back({a, b, c});
    }
  }

  std::sort(triangles.begin(), triangles.end());
  auto end = std::unique_copy(triangles.begin(), triangles.end(), triangles.begin());

  math::field3i boundary_triangles(3, end - triangles.begin());
  for (idx i = 0; i < boundary_triangles.cols(); ++i) {
    boundary_triangles(0, i) = std::get<0>(triangles[i]);
    boundary_triangles(1, i) = std::get<1>(triangles[i]);
    boundary_triangles(2, i) = std::get<2>(triangles[i]);
  }
  return boundary_triangles;
}

math::field3i fix_boundary_orientation(math::field3r const& vertices, math::field4i const& tetrahedrons,
                                       math::field3i const& boundary_triangles) {
  CHECK(false) << "Not Implemented Error.";
}

}  // namespace ax::geo
