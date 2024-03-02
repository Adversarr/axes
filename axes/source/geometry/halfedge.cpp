//
// Created by Yang Jerry on 2024/3/1.
//
#include "axes/geometry/halfedge.hpp"

#include <entt/entity/entity.hpp>

namespace ax::geo {

void establish_pair(HalfedgeEdge_t* e1, HalfedgeEdge_t* e2) {
  std::tie(e1->pair_, e2->pair_) = {e2, e1};
}

HalfedgeMesh::HalfedgeMesh(math::field3r const& vertices, math::field3i const& indices) {
  for (idx i = 0; i < vertices.cols(); ++i) {
    vertices_.emplace_back(std::make_unique<HalfedgeVertex_t>(vertices.col(i), i));
  }
  for (auto ijk : math::each(indices)) {
    const idx i = ijk.x(), j = ijk.y(), k = ijk.z();
    auto fijk = std::make_unique<HalfedgeFace_t>();
    auto e_ij = std::make_unique<HalfedgeEdge_t>();
    auto e_jk = std::make_unique<HalfedgeEdge_t>();
    auto e_ki = std::make_unique<HalfedgeEdge_t>();

    // Make the face point to one of its edges
    fijk->halfedge_entry_ = e_ij.get();
    e_ki->face_ = e_jk->face_ = e_ij->face_ = fijk.get();

    // Make ij -> jk -> ki -> ij
    e_ij->next_ = e_jk.get();
    e_ij->prev_ = e_ki.get();
    e_ij->vertex_ = vertices_[j].get();
    e_jk->next_ = e_ki.get();
    e_jk->prev_ = e_ij.get();
    e_jk->vertex_ = vertices_[k].get();
    e_ki->next_ = e_ij.get();
    e_ki->prev_ = e_jk.get();
    e_ki->vertex_ = vertices_[i].get();

    vertices_[i]->halfedge_entry_ = e_ki.get();
    vertices_[j]->halfedge_entry_ = e_ij.get();
    vertices_[k]->halfedge_entry_ = e_jk.get();

    // Move the unique pointers into the mesh
    faces_.push_back(std::move(fijk));
    edges_.push_back(std::move(e_ij));
    edges_.push_back(std::move(e_jk));
    edges_.push_back(std::move(e_ki));
  }

  std::map<std::pair<HalfedgeVertex_t*, HalfedgeVertex_t*>, HalfedgeEdge_t*> edge_map;
  for (auto [id, e] : utils::enumerate(edges_)) {
    auto v = e->vertex_;
    auto next_v = e->next_->vertex_;
    auto pair = std::make_pair(v, next_v);
    if (auto it = edge_map.find(pair); it != edge_map.end()) {
      establish_pair(it->second, e.get());
      edge_map.erase(it);
    } else {
      edge_map.emplace(std::make_pair(next_v, v), e.get());
    }
  }

  // For those on the boundary, fix the pair:
  for (auto& e : edges_) {
    if (e->pair_ == nullptr) {
      auto new_edge = std::make_unique<HalfedgeEdge_t>();
      new_edge->vertex_ = e->next_->next_->vertex_;
      new_edge->face_ = e->face_;
      establish_pair(e.get(), new_edge.get());
    }
  }

  for (auto& e : edges_) {
    if (e->next_ == nullptr) {
      for (auto e2 = e->pair_; e2 != e.get(); e2 = e2->next_->pair_) {
        if (e2->prev_ == nullptr) {
          e->next_ = e2;
          e2->prev_ = e.get();
          break;
        }
      }
    }
  }

  // Check all edge has pair:
  for (auto [id, e] : utils::enumerate(edges_)) {
    AX_CHECK(e->prev_ != nullptr);
    AX_CHECK(e->next_ != nullptr);
    AX_CHECK(e->face_ != nullptr);
    AX_CHECK(e->vertex_ != nullptr);
    AX_CHECK(e->pair_ != nullptr);
  }

  for (auto [id, v] : utils::enumerate(vertices_)) {
    AX_CHECK(v->halfedge_entry_ != nullptr);
  }

  for (auto [id, f] : utils::enumerate(faces_)) {
    AX_CHECK(f->halfedge_entry_ != nullptr);
  }
}

std::pair<math::field3r, math::field3i> HalfedgeMesh::ToTriangleMesh() const {
  math::field3i indices(3, faces_.size());
  math::field3r vertices(3, vertices_.size());

  std::map<HalfedgeVertex_t*, idx> vertex_map;
  for (auto [id, v] : utils::enumerate(vertices_)) {
    vertex_map.emplace(v.get(), id);
    vertices.col(id) = v->position_;
  }

  for (auto [id, face] : utils::enumerate(faces_)) {
    auto e = face->halfedge_entry_;
    math::vec3i ijk;
    for (idx i = 0; i < 3; ++i) {
      if (auto it = vertex_map.find(e->vertex_); it == vertex_map.end()) {
        AX_LOG(INFO) << "Face: " << id;
        AX_LOG(INFO) << "Edge: " << e;
        AX_LOG(INFO) << "Edge->Vertex: " << e->vertex_;
        AX_LOG(INFO) << "Edge->Next: " << e->next_;
        AX_LOG(INFO) << "Edge->Prev: " << e->prev_;
        AX_LOG(INFO) << "Edge->Pair: " << e->pair_;
        AX_LOG(INFO) << "Edge->Face: " << e->face_;
      } else {
        ijk[i] = vertex_map.at(e->vertex_);
      }

      e = e->next_;
    }
    indices.col(id) = ijk;
  }
  return std::make_pair(vertices, indices);
}

void HalfedgeMesh::RemoveEdgeInternal(HalfedgeEdge_t* edge) {
  auto it = std::find_if(edges_.begin(), edges_.end(),
                         [edge](auto const& e) { return e.get() == edge; });
  AX_LOG(INFO) << "Removing edge: " << edge;
  AX_CHECK(it != edges_.end());

  std::swap(*it, edges_.back());

  if (edge->vertex_->halfedge_entry_ == edge) {
    edge->vertex_->halfedge_entry_ = edge->next_->pair_;
  }

  edges_.pop_back();
}

void HalfedgeMesh::RemoveFaceInternal(HalfedgeFace_t* face) {
  auto it = std::find_if(faces_.begin(), faces_.end(),
                         [face](auto const& f) { return f.get() == face; });
  AX_LOG(INFO) << "Removing face: " << face;
  std::swap(*it, faces_.back());
  faces_.pop_back();
}

void HalfedgeMesh::CollapseEdge(HalfedgeEdge_t* edge, math::vec3r const& target_position) {
  /**
   *    a
   *  /  \
   * c--->b
   * \  /
   *  d */

  // The vertex to collapse to
  HalfedgeVertex_t* head_vertex = edge->vertex_;  // b
  head_vertex->position_= target_position;
  HalfedgeEdge_t* pair = edge->pair_;
  HalfedgeVertex_t* tail_vertex = pair->vertex_;  // c
  // Assign all c to b.
  ForeachEdgeAroundVertex(tail_vertex,
                          [head_vertex](HalfedgeEdge_t* e) { e->vertex_ = head_vertex; });

  // Fix the pair of the edge to be removed
  edge->next_->pair_->pair_ = edge->prev_->pair_;
  edge->prev_->pair_->pair_ = edge->next_->pair_;
  pair->prev_->pair_->pair_ = pair->next_->pair_;

  // Do remove the edge
  RemoveFaceInternal(edge->face_);
  RemoveFaceInternal(pair->face_);
  RemoveEdgeInternal(pair->next_);
  RemoveEdgeInternal(edge->prev_);
  RemoveEdgeInternal(pair);
  RemoveEdgeInternal(edge);
  RemoveVertexInternal(tail_vertex);
}

bool HalfedgeMesh::CheckCollapse(HalfedgeEdge_t* edge) {
  // $p, q$ are boundaries, but $(p, q)$ is not a boundary edge.
  HalfedgeEdge_t* p_bd_edge = nullptr;
  HalfedgeEdge_t* q_bd_edge = nullptr;
  ForeachEdgeAroundVertex(edge->vertex_, [&p_bd_edge](HalfedgeEdge_t* e) {
    if (e->pair_->face_ == nullptr) {
      p_bd_edge = e;
    }
  });
  ForeachEdgeAroundVertex(edge->pair_->vertex_, [&q_bd_edge](HalfedgeEdge_t* e) {
    if (e->pair_->face_ == nullptr) {
      q_bd_edge = e;
    }
  });

  if (p_bd_edge != nullptr && q_bd_edge != nullptr && p_bd_edge != edge
      && q_bd_edge != edge->pair_) {
    return false;
  }

  // Foreach $k$ incident to both i and j, $(i,j,k)$ should be the vertices of a triangle.
  std::set<HalfedgeVertex_t*> i_incidents;
  std::set<HalfedgeVertex_t*> j_incidents;
  ForeachEdgeAroundVertex(
      edge->vertex_, [&i_incidents](HalfedgeEdge_t* e) { i_incidents.emplace(e->pair_->vertex_); });

  std::vector<HalfedgeVertex_t*> incidents_both;

  ForeachEdgeAroundVertex(edge->pair_->vertex_, [&i_incidents, &incidents_both](HalfedgeEdge_t* e) {
    if (i_incidents.contains(e->pair_->vertex_)) {
      incidents_both.push_back(e->pair_->vertex_);
    }
  });

  auto head = edge->vertex_;
  auto tail = edge->pair_->vertex_;

  // Check if <head, tail, k> is a triangle foreach k in incidents_both
  for (auto k : incidents_both) {
    if (k == head || k == tail) {
      continue;
    }
    if (auto* e = TryGetEdgeBetween(head, k); e == nullptr) {
      return false;
    } else if (auto* e = TryGetEdgeBetween(tail, k); e == nullptr) {
      return false;
    }
  }

  return true;
}

HalfedgeEdge_t* HalfedgeMesh::TryGetEdgeBetween(HalfedgeVertex_t* v1, HalfedgeVertex_t* v2) {
  HalfedgeEdge_t* edge = nullptr;
  ForeachEdgeAroundVertex(v2, [&](auto* e) {
    if (e->pair_->vertex_ == v1) {
      edge = e;
    }
  });

  return edge;
}

void HalfedgeMesh::ForeachEdgeAroundVertex(HalfedgeVertex_t* vert,
                                           std::function<void(HalfedgeEdge_t*)> const& fn) {
  auto beg = vert->halfedge_entry_;
  idx rec_guard = 100;
  do {
    fn(beg);
    beg = beg->pair_->next_;
    AX_CHECK(rec_guard-- > 0) << "This vertex has too many edges!";
  } while (beg != vert->halfedge_entry_);
}

void HalfedgeMesh::ForeachEdge(std::function<void(HalfedgeEdge_t*)> const& fn) {
  for (auto const& e : edges_) {
    fn(e.get());
  }
}

void HalfedgeMesh::ForeachVertex(std::function<void(HalfedgeVertex_t*)> const& fn) {
  for (auto const& v : vertices_) {
    fn(v.get());
  }
}

void HalfedgeMesh::RemoveVertexInternal(HalfedgeVertex_t* vert) {
  auto it = std::find_if(vertices_.begin(), vertices_.end(),
                         [vert](auto const& v) { return v.get() == vert; });
  AX_LOG(INFO) << "Removing vertex: " << vert;
  AX_CHECK(it != vertices_.end());
  std::swap(*it, vertices_.back());
  vertices_.pop_back();
}
}  // namespace ax::geo