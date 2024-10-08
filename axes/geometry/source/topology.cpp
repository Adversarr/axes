#include "ax/geometry/topology.hpp"
#include "ax/math/linalg.hpp"
#include <set>

namespace ax::geo {

math::IndexField2 get_edges(math::IndexField3 const& triangles) {
  std::vector<utils::DupTuple<Index, 2>> edges;
  edges.reserve(triangles.cols() * 3);
  for (Index i = 0; i < triangles.cols(); ++i) {
    for (Index j = 0; j < 3; ++j) {
      Index a = triangles(j, i);
      Index b = triangles((j + 1) % 3, i);
      if (a > b) std::swap(a, b);
      edges.emplace_back(a, b);
    }
  }

  std::sort(edges.begin(), edges.end());
  auto end = std::unique_copy(edges.begin(), edges.end(), edges.begin());

  math::IndexField2 edges_field(2, end - edges.begin());
  for (Index i = 0; i < edges_field.cols(); ++i) {
    edges_field(0, i) = std::get<0>(edges[static_cast<size_t>(i)]);
    edges_field(1, i) = std::get<1>(edges[static_cast<size_t>(i)]);
  }
  return edges_field;
}

math::IndexField2 get_edges(math::IndexField4 const& tetrahedrons) {
  std::vector<utils::DupTuple<Index, 2>> edges;
  edges.reserve(static_cast<size_t>(tetrahedrons.cols() * 6));
  for (Index i = 0; i < tetrahedrons.cols(); ++i) {
    for (Index j = 0; j < 4; ++j) {
      for (Index k = j + 1; k < 4; ++k) {
        Index a = tetrahedrons(j, i);
        Index b = tetrahedrons(k, i);
        if (a > b) std::swap(a, b);
        edges.emplace_back(a, b);
      }
    }
  }

  std::sort(edges.begin(), edges.end());
  auto end = std::unique_copy(edges.begin(), edges.end(), edges.begin());

  math::IndexField2 edges_field(2, end - edges.begin());
  for (Index i = 0; i < edges_field.cols(); ++i) {
    edges_field(0, i) = std::get<0>(edges[static_cast<size_t>(i)]);
    edges_field(1, i) = std::get<1>(edges[static_cast<size_t>(i)]);
  }
  return edges_field;
}

math::IndexField2 get_boundary_edges(math::IndexField3 const& triangles) {
  std::vector<utils::DupTuple<Index, 2>> edges;

  for (Index i = 0; i < triangles.cols(); ++i) {
    for (Index j = 0; j < 3; ++j) {
      Index a = triangles(j, i);
      Index b = triangles((j + 1) % 3, i);
      if (a > b) std::swap(a, b);
      edges.emplace_back(a, b);
    }
  }

  std::sort(edges.begin(), edges.end());
  auto end = std::unique_copy(edges.begin(), edges.end(), edges.begin());

  math::IndexField2 boundary_edges(2, end - edges.begin());
  for (Index i = 0; i < boundary_edges.cols(); ++i) {
    boundary_edges(0, i) = std::get<0>(edges[i]);
    boundary_edges(1, i) = std::get<1>(edges[i]);
  }
  return boundary_edges;
}

math::IndexField2 get_boundary_edges(math::RealField3 const&, math::IndexField4 const& tetrahedrons) {
  std::set<std::tuple<Index, Index, Index>> triangles;
  auto put_triangle = [&](Index a, Index b, Index c) {
    if (a > b) std::swap(a, b);
    if (a > c) std::swap(a, c);
    if (b > c) std::swap(b, c);
    if (triangles.find({a, b, c}) != triangles.end()) {
      triangles.erase({a, b, c});
    } else {
      triangles.insert({a, b, c});
    }
  };

  for (Index i = 0; i < tetrahedrons.cols(); ++i) {
    put_triangle(tetrahedrons(0, i), tetrahedrons(1, i), tetrahedrons(2, i));
    put_triangle(tetrahedrons(0, i), tetrahedrons(1, i), tetrahedrons(3, i));
    put_triangle(tetrahedrons(0, i), tetrahedrons(2, i), tetrahedrons(3, i));
    put_triangle(tetrahedrons(1, i), tetrahedrons(2, i), tetrahedrons(3, i));
  }

  std::set<std::pair<Index, Index>> edges;
  auto put_edge = [&](Index a, Index b) {
    if (a > b) std::swap(a, b);
    edges.insert({a, b});
  };

  for (auto const& [a, b, c] : triangles) {
    put_edge(a, b);
    put_edge(b, c);
    put_edge(c, a);
  }

  math::IndexField2 boundary_edges(2, edges.size());
  Index i = 0;
  for (auto const& [a, b] : edges) {
    boundary_edges(0, i) = a;
    boundary_edges(1, i) = b;
    ++i;
  }

  return boundary_edges;
}

math::IndexField3 get_boundary_triangles(math::RealField3 const& vertices,
  math::IndexField4 const& tetrahedrons) {
  std::vector<utils::DupTuple<Index, 3>> triangles;

  for (Index i = 0; i < tetrahedrons.cols(); ++i) {
    for (Index j = 0; j < 4; ++j) {
      Index o = tetrahedrons(j, i);
      Index a = tetrahedrons((j + 1) % 4, i);
      Index b = tetrahedrons((j + 2) % 4, i);
      Index c = tetrahedrons((j + 3) % 4, i);
      // check det[oabc]
      math::RealVector3 oa = vertices.col(a) - vertices.col(o);
      math::RealVector3 ob = vertices.col(b) - vertices.col(o);
      math::RealVector3 oc = vertices.col(c) - vertices.col(o);
      if (math::cross(oa, ob).dot(oc) > 0) {
        triangles.push_back({a, b, c});
      } else {
        triangles.push_back({b, a, c});
      }
    }
  }

  auto equal = [](auto const& a, auto const& b) -> bool {
    Index min_a = std::min({std::get<0>(a), std::get<1>(a), std::get<2>(a)});
    Index min_b = std::min({std::get<0>(b), std::get<1>(b), std::get<2>(b)});
    Index max_a = std::max({std::get<0>(a), std::get<1>(a), std::get<2>(a)});
    Index max_b = std::max({std::get<0>(b), std::get<1>(b), std::get<2>(b)});
    Index mid_a = std::get<0>(a) + std::get<1>(a) + std::get<2>(a) - min_a - max_a;
    Index mid_b = std::get<0>(b) + std::get<1>(b) + std::get<2>(b) - min_b - max_b;
    if (min_a != min_b) return false;
    if (max_a != max_b) return false;
    return mid_a == mid_b;
  };

  auto less = [](auto const& a, auto const& b) -> bool {
    Index min_a = std::min({std::get<0>(a), std::get<1>(a), std::get<2>(a)});
    Index min_b = std::min({std::get<0>(b), std::get<1>(b), std::get<2>(b)});
    if (min_a != min_b) return min_a < min_b;
    Index max_a = std::max({std::get<0>(a), std::get<1>(a), std::get<2>(a)});
    Index max_b = std::max({std::get<0>(b), std::get<1>(b), std::get<2>(b)});
    Index mid_a = std::get<0>(a) + std::get<1>(a) + std::get<2>(a) - min_a - max_a;
    Index mid_b = std::get<0>(b) + std::get<1>(b) + std::get<2>(b) - min_b - max_b;
    if (mid_a != mid_b) return mid_a < mid_b;
    return max_a < max_b;
  };

  std::sort(triangles.begin(), triangles.end(), less);
  std::vector<utils::DupTuple<Index, 3>> unique;
  unique.reserve(triangles.size() / 4);
  for (size_t i = 1; i < triangles.size(); ++i) {
    if (!equal(triangles[i], triangles[i - 1])) {
      unique.push_back(triangles[i - 1]);
    } else {
      ++i;
    }
  }

  math::IndexField3 boundary_triangles(3, unique.size());
  for (Index i = 0; i < boundary_triangles.cols(); ++i) {
    boundary_triangles(0, i) = std::get<0>(unique[i]);
    boundary_triangles(1, i) = std::get<1>(unique[i]);
    boundary_triangles(2, i) = std::get<2>(unique[i]);
  }

  return boundary_triangles;
}

}  // namespace ax::geo
