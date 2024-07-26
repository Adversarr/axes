#include "ax/geometry/topology.hpp"
#include "ax/math/linalg.hpp"

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
    edges_field(0, i) = std::get<0>(edges[static_cast<size_t>(i)]);
    edges_field(1, i) = std::get<1>(edges[static_cast<size_t>(i)]);
  }
  return edges_field;
}

math::field2i get_edges(math::field4i const& tetrahedrons) {
  std::vector<utils::DupTuple<idx, 2>> edges;
  edges.reserve(static_cast<size_t>(tetrahedrons.cols() * 6));
  for (idx i = 0; i < tetrahedrons.cols(); ++i) {
    for (idx j = 0; j < 4; ++j) {
      for (idx k = j + 1; k < 4; ++k) {
        idx a = tetrahedrons(j, i);
        idx b = tetrahedrons(k, i);
        if (a > b) std::swap(a, b);
        edges.emplace_back(a, b);
      }
    }
  }

  std::sort(edges.begin(), edges.end());
  auto end = std::unique_copy(edges.begin(), edges.end(), edges.begin());

  math::field2i edges_field(2, end - edges.begin());
  for (idx i = 0; i < edges_field.cols(); ++i) {
    edges_field(0, i) = std::get<0>(edges[static_cast<size_t>(i)]);
    edges_field(1, i) = std::get<1>(edges[static_cast<size_t>(i)]);
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

math::field2i get_boundary_edges(math::field3r const&, math::field4i const& tetrahedrons) {
  std::set<std::tuple<idx, idx, idx>> triangles;
  auto put_triangle = [&](idx a, idx b, idx c) {
    if (a > b) std::swap(a, b);
    if (a > c) std::swap(a, c);
    if (b > c) std::swap(b, c);
    if (triangles.find({a, b, c}) != triangles.end()) {
      triangles.erase({a, b, c});
    } else {
      triangles.insert({a, b, c});
    }
  };

  for (idx i = 0; i < tetrahedrons.cols(); ++i) {
    put_triangle(tetrahedrons(0, i), tetrahedrons(1, i), tetrahedrons(2, i));
    put_triangle(tetrahedrons(0, i), tetrahedrons(1, i), tetrahedrons(3, i));
    put_triangle(tetrahedrons(0, i), tetrahedrons(2, i), tetrahedrons(3, i));
    put_triangle(tetrahedrons(1, i), tetrahedrons(2, i), tetrahedrons(3, i));
  }

  std::set<std::pair<idx, idx>> edges;
  auto put_edge = [&](idx a, idx b) {
    if (a > b) std::swap(a, b);
    edges.insert({a, b});
  };

  for (auto const& [a, b, c] : triangles) {
    put_edge(a, b);
    put_edge(b, c);
    put_edge(c, a);
  }

  math::field2i boundary_edges(2, edges.size());
  idx i = 0;
  for (auto const& [a, b] : edges) {
    boundary_edges(0, i) = a;
    boundary_edges(1, i) = b;
    ++i;
  }

  return boundary_edges;
}

math::field3i get_boundary_triangles(math::field3r const& vertices, 
  math::field4i const& tetrahedrons) {
  std::vector<utils::DupTuple<idx, 3>> triangles;

  for (idx i = 0; i < tetrahedrons.cols(); ++i) {
    for (idx j = 0; j < 4; ++j) {
      idx o = tetrahedrons(j, i);
      idx a = tetrahedrons((j + 1) % 4, i);
      idx b = tetrahedrons((j + 2) % 4, i);
      idx c = tetrahedrons((j + 3) % 4, i);
      // check det[oabc]
      math::vec3r oa = vertices.col(a) - vertices.col(o);
      math::vec3r ob = vertices.col(b) - vertices.col(o);
      math::vec3r oc = vertices.col(c) - vertices.col(o);
      if (math::cross(oa, ob).dot(oc) > 0) {
        triangles.push_back({a, b, c});
      } else {
        triangles.push_back({b, a, c});
      }
    }
  }

  auto equal = [](auto const& a, auto const& b) -> bool {
    idx min_a = std::min({std::get<0>(a), std::get<1>(a), std::get<2>(a)});
    idx min_b = std::min({std::get<0>(b), std::get<1>(b), std::get<2>(b)});
    idx max_a = std::max({std::get<0>(a), std::get<1>(a), std::get<2>(a)});
    idx max_b = std::max({std::get<0>(b), std::get<1>(b), std::get<2>(b)});
    idx mid_a = std::get<0>(a) + std::get<1>(a) + std::get<2>(a) - min_a - max_a;
    idx mid_b = std::get<0>(b) + std::get<1>(b) + std::get<2>(b) - min_b - max_b;
    if (min_a != min_b) return false;
    if (max_a != max_b) return false;
    return mid_a == mid_b;
  };

  auto less = [](auto const& a, auto const& b) -> bool {
    idx min_a = std::min({std::get<0>(a), std::get<1>(a), std::get<2>(a)});
    idx min_b = std::min({std::get<0>(b), std::get<1>(b), std::get<2>(b)});
    if (min_a != min_b) return min_a < min_b;
    idx max_a = std::max({std::get<0>(a), std::get<1>(a), std::get<2>(a)});
    idx max_b = std::max({std::get<0>(b), std::get<1>(b), std::get<2>(b)});
    idx mid_a = std::get<0>(a) + std::get<1>(a) + std::get<2>(a) - min_a - max_a;
    idx mid_b = std::get<0>(b) + std::get<1>(b) + std::get<2>(b) - min_b - max_b;
    if (mid_a != mid_b) return mid_a < mid_b;
    return max_a < max_b;
  };

  std::sort(triangles.begin(), triangles.end(), less);
  std::vector<utils::DupTuple<idx, 3>> unique;
  unique.reserve(triangles.size() / 4);
  for (size_t i = 1; i < triangles.size(); ++i) {
    if (!equal(triangles[i], triangles[i - 1])) {
      unique.push_back(triangles[i - 1]);
    } else {
      ++i;
    }
  }

  math::field3i boundary_triangles(3, unique.size());
  for (idx i = 0; i < boundary_triangles.cols(); ++i) {
    boundary_triangles(0, i) = std::get<0>(unique[i]);
    boundary_triangles(1, i) = std::get<1>(unique[i]);
    boundary_triangles(2, i) = std::get<2>(unique[i]);
  }

  return boundary_triangles;
}

}  // namespace ax::geo
