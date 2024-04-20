#include "solver.hpp"
#include "ax/core/echo.hpp"
#include "ax/geometry/halfedge.hpp"
#include <queue>
#include <set>

Dijkstra::Dijkstra(geo::SurfaceMesh mesh) : mesh_(std::move(mesh)) {
  // Initialize the edge flipper
  adj_list_.resize(mesh_.vertices_.cols(), {});
  for (idx i = 0; i < mesh_.indices_.cols(); ++i) {
    for (idx j = 0; j < 3; ++j) {
      idx u = mesh_.indices_(j, i);
      idx v = mesh_.indices_((j + 1) % 3, i);
      adj_list_[u].push_back(v);
      adj_list_[v].push_back(u);
    }
  }

  for (idx i = 0; i < mesh_.vertices_.cols(); ++i) {
    std::sort(adj_list_[i].begin(), adj_list_[i].end());
    adj_list_[i].erase(std::unique(adj_list_[i].begin(), adj_list_[i].end()), adj_list_[i].end());
  }
}

struct LESS_FOR_PATH {
  LESS_FOR_PATH(math::field1r& distances) : distances_(distances) {}
  bool operator()(idx l, idx r) const {
    return distances_(l) > distances_(r);
  }
  math::field1r& distances_;
};

Path Dijkstra::ShortestPath(idx start, idx end) {
  // Find the shortest path by Dijkstra's algorithm
  // and return the path as a list of PathItem

  Path path;
  math::field1r distances(1, mesh_.vertices_.cols());
  math::field1i from(1, mesh_.vertices_.cols());
  math::field1i visited(1, mesh_.vertices_.cols());
  distances.setConstant(math::inf<>);
  visited.setConstant(0);
  from.setConstant(-1);
  // Initialize the distances
  distances(start) = 0;
  // Initialize the priority queue
  std::priority_queue<idx, std::vector<idx>, LESS_FOR_PATH> pq(LESS_FOR_PATH{distances});
  pq.push(start);
  while (!pq.empty()) {
    idx u = pq.top();
    pq.pop();
    if (u == end) {
      break;
    }
    if (visited(u)) {
      continue;
    }
    visited(u) = 1;

    for (idx v : adj_list_[u]) {
      if (visited(v)) {
        continue;
      }
      real new_dist = distances(u) + (mesh_.vertices_.col(u) - mesh_.vertices_.col(v)).norm();
      if (new_dist < distances(v)) {
        distances(v) = new_dist;
        from(v) = u;
        pq.push(v);
      }
    }
  }

  // Reconstruct the path
  idx u = end;
  while (u != -1) {
    path.push_back({u, false, 0, 0});
    u = from(u);
  }
  return path;
}

struct Widge {
  bool valid;
  idx center_vertex;
  std::set<idx> associated_vertices;
  std::vector<idx> associated_faces;
  Path original_path;
  geo::HalfedgeMesh* mesh;
  Widge() : valid(false), center_vertex(0) {}
};

// 1. unfold. mesh -> path -> va -> vb -> vc -> [Widge]
// 2. flip edge. [Widge] -> Path

Widge unfold(geo::HalfedgeMesh mesh, Path p0, idx va_in_path, idx vb_in_path, idx vc_in_path) {
  // Unfold the mesh to a widge
  Widge w;
  w.mesh = &mesh;
  idx va = p0[va_in_path].id_;
  idx vb = p0[vb_in_path].id_;
  idx vc = p0[vc_in_path].id_;
  auto A = mesh.GetVertex(va); AX_CHECK(A);
  auto B = mesh.GetVertex(vb); AX_CHECK(B);
  auto C = mesh.GetVertex(vc); AX_CHECK(C);
  auto edge_ab = mesh.TryGetEdgeBetween(A.vertex_, B.vertex_); AX_CHECK(edge_ab);
  auto edge_bc = mesh.TryGetEdgeBetween(B.vertex_, C.vertex_); AX_CHECK(edge_bc);
  // 1. First you have to find the correct faces between them. and find the correct 
  //    vertices and faces around them.
  // Suppose you have two vertices fixed, i.e. A B, we wish to find the correct face
  // between them. We can do this by checking the all the faces around A and B.

  math::vec2r ab{(B->position_ - A->position_).norm(), 0};
  auto rel_ab = [&] (math::vec3r const& p) {
    math::vec2r pa = p - A->position_;
    real x = pa.dot(B->position_ - A->position_) / ab(0);
    real y = sqrt(pa.squaredNorm() - x * x);
    return math::vec2r{x, y};
  };

  std::vector<math::vec2r> face_vertices;
  auto it = A.begin();
  for (; it != A.end(); ++it) {
    auto edge = *it;
    // Determine the position.
  }
}

Path make_shortest_geodesic(geo::SurfaceMesh mesh, Path p0) {
  // By flip edge.
  // Initialize the edge flipper
  geo::HalfedgeMesh he_mesh(mesh.vertices_, mesh.indices_);
  Path sol = p0; //< Current solution
  if (sol.size() <= 2) {
    return sol;
  }
  idx start = p0.back().id_;
  idx end = p0.front().id_;
}