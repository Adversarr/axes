//
// Created by Yang Jerry on 2024/3/1.
//
#include "ax/geometry/halfedge.hpp"

#include <entt/entity/entity.hpp>

#include "ax/geometry/common.hpp"
#include "ax/core/echo.hpp"
#include <map>
#include <absl/container/flat_hash_map.h>
#include <absl/container/flat_hash_set.h>
#include <absl/container/inlined_vector.h>

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
    e_ij->pair_ = e_jk->pair_ = e_ki->pair_ = nullptr;

    vertices_[i]->halfedge_entry_ = e_ki.get();
    vertices_[j]->halfedge_entry_ = e_ij.get();
    vertices_[k]->halfedge_entry_ = e_jk.get();

    // Move the unique pointers into the mesh
    faces_.emplace_back(std::move(fijk));
    edges_.emplace_back(std::move(e_ij));
    edges_.emplace_back(std::move(e_jk));
    edges_.emplace_back(std::move(e_ki));
  }

  std::map<std::pair<HalfedgeVertex_t*, HalfedgeVertex_t*>, HalfedgeEdge_t*> edge_map;
  for (auto [id, e] : utils::enumerate(edges_)) {
    auto [head, tail] = e->HeadAndTail();
    auto ht = std::make_pair(head, tail);
    if (auto it = edge_map.find(std::make_pair(tail, head)); it != edge_map.end()) {
      establish_pair(e.get(), it->second);
      edge_map.erase(it);
    } else {
      edge_map.emplace(ht, e.get());
    }
  }

  // For those on the boundary, fix the pair:
  size_t interior_edges = edges_.size();
  for (size_t i = 0; i < interior_edges; ++i) {
    auto& e = edges_[i];
    if (e->pair_ == nullptr) {
      auto new_edge = std::make_unique<HalfedgeEdge_t>();
      new_edge->vertex_ = e->Tail();
      new_edge->face_ = nullptr;
      establish_pair(e.get(), new_edge.get());
      edges_.emplace_back(std::move(new_edge));
    }
  }

  for (auto& e : edges_) {
    if (e->next_ == nullptr) {
      AX_DCHECK(e->IsBoundary()) << "Non-Boundary edge should have `next` initialized.";
      bool found = false;
      for (auto e2 = e->pair_; e2 != e.get(); e2 = e2->prev_->pair_) {
        if (e2->prev_ == nullptr) {
          e->next_ = e2;
          e2->prev_ = e.get();
          found = true;
          break;
        }
      }
      AX_DCHECK(found) << "No next edge found for boundary edge: " << e;
    }
  }

  // Check all edge has pair:
  for (auto [id, e] : utils::enumerate(edges_)) {
    AX_CHECK(e->prev_ != nullptr);
    AX_CHECK(e->next_ != nullptr);
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

SurfaceMesh HalfedgeMesh::ToTriangleMesh() const {
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
        AX_LOG(ERROR) << "Face: " << id;
        AX_LOG(ERROR) << "Edge: " << e;
        AX_LOG(ERROR) << "Edge->Vertex: " << e->vertex_;
        AX_LOG(ERROR) << "Edge->Next: " << e->next_;
        AX_LOG(ERROR) << "Edge->Prev: " << e->prev_;
        AX_LOG(ERROR) << "Edge->Pair: " << e->pair_;
        AX_LOG(ERROR) << "Edge->Face: " << e->face_;
      } else {
        ijk[i] = vertex_map.at(e->vertex_);
      }

      e = e->next_;
    }
    indices.col(id) = ijk;
  }
  return SurfaceMesh{vertices, indices};
}
HalfedgeFaceHandle HalfedgeMesh::GetFace(idx idx) const {
  for (auto const& f : faces_) {
    if (f->original_id_ == idx) {
      return HalfedgeFaceHandle{f.get()};
    }
  }
  return HalfedgeFaceHandle{nullptr};
}

void HalfedgeMesh::RemoveEdgeInternal(HalfedgeEdge_t* edge) {
  auto it = std::find_if(edges_.begin(), edges_.end(),
                         [edge](auto const& e) { return e.get() == edge; });
  AX_DCHECK(it != edges_.end());
  std::swap(*it, edges_.back());
  edges_.pop_back();
}

void HalfedgeMesh::RemoveFaceInternal(HalfedgeFace_t* face) {
  auto it = std::find_if(faces_.begin(), faces_.end(),
                         [face](auto const& f) { return f.get() == face; });
  AX_DCHECK(it != faces_.end());
  std::swap(*it, faces_.back());
  faces_.pop_back();
}

void HalfedgeMesh::RemoveVertexInternal(HalfedgeVertex_t* vert) {
  auto it = std::find_if(vertices_.begin(), vertices_.end(),
                         [vert](auto const& v) { return v.get() == vert; });
  AX_DCHECK(it != vertices_.end());
  std::swap(*it, vertices_.back());

#ifndef NDEBUG
  auto ite = std::find_if(edges_.begin(), edges_.end(),
                         [vert](auto const& e) { return e->Head() == vert; });
  AX_DCHECK(ite == edges_.end())
    << "After you delete the vertex, there should be no edge pointing to it." << ite->get();
#endif
  vertices_.pop_back();
}

HalfedgeVertexHandle HalfedgeMesh::GetVertex(idx idx) const {
  for (auto const& v : vertices_) {
    if (v->original_id_ == idx) {
      return HalfedgeVertexHandle{v.get()};
    }
  }
  return HalfedgeVertexHandle{nullptr};
}
void HalfedgeMesh::CollapseEdge(HalfedgeEdge_t* edge, math::vec3r const& target_position) {
  /**
   *    a
   *  /  \
   * c--->b
   * \  /
   *  d */
  AX_DCHECK(std::find_if(edges_.begin(), edges_.end(), [edge](const auto& e) {
    return e.get() == edge;
  }) != edges_.end());

  // The vertex to collapse to
  auto head_vertex = edge->Head();
  head_vertex->position_= target_position;
  HalfedgeEdge_t* pair = edge->pair_;
  auto tail_vertex = edge->Tail();
  AX_DCHECK(head_vertex != tail_vertex);
  // Assign all c to b.
  ForeachEdgeAroundVertex(tail_vertex,
                          [head_vertex](HalfedgeEdge_t* e) {
                            e->vertex_ = head_vertex;
                          });

  head_vertex->halfedge_entry_ = edge->next_->pair_;
  // AX_DLOG(INFO) << "Assigning entry[" << head_vertex << "] <- " << edge->next_->pair_;
  if (!edge->IsBoundary()) {
    edge->next_->vertex_->halfedge_entry_ = edge->prev_->pair_;
    // AX_DLOG(INFO) << "Assigning entry[" << edge->next_->vertex_ << "] <- " << edge->prev_->pair_;
  }
  if (!edge->pair_->IsBoundary()) {
    edge->pair_->next_->vertex_->halfedge_entry_ = edge->pair_->prev_->pair_;
    // AX_DLOG(INFO) << "Assigning entry[" << edge->pair_->next_->vertex_ << "] <- " << edge->pair_->prev_->pair_;
  }

  // Fix the pair of the edge to be removed
  if (!edge->IsBoundary()) {
    establish_pair(edge->next_->pair_, edge->prev_->pair_);
  } else {
    std::tie(edge->next_->prev_, edge->prev_->next_) = {edge->prev_, edge->next_};
  }
  if (!pair->IsBoundary()) {
    establish_pair(pair->next_->pair_, pair->prev_->pair_);
  } else {
    std::tie(pair->next_->prev_, pair->prev_->next_) = {pair->prev_, pair->next_};
  }


  // Do remove the edge
  if (edge->face_ != nullptr) {
    RemoveFaceInternal(edge->face_);
    RemoveEdgeInternal(edge->prev_);
    RemoveEdgeInternal(edge->next_);
  }
  if (pair->face_ != nullptr) {
    RemoveFaceInternal(pair->face_);
    RemoveEdgeInternal(pair->prev_);
    RemoveEdgeInternal(pair->next_);
  }
  RemoveEdgeInternal(edge);
  RemoveEdgeInternal(pair);
  RemoveVertexInternal(tail_vertex);
}

bool HalfedgeMesh::CheckCollapse(HalfedgeEdge_t* edge) {
  if (edge->IsBoundary()) {
    return false;
  }
  // $p, q$ are boundaries, but $(p, q)$ is not a boundary edge.
  HalfedgeEdge_t* p_bd_edge = nullptr;
  HalfedgeEdge_t* q_bd_edge = nullptr;
  auto [head, tail] = edge->HeadAndTail();
  ForeachEdgeAroundVertex(head, [&p_bd_edge](HalfedgeEdge_t* e) {
    if (e->pair_->face_ == nullptr) {
      p_bd_edge = e;
    }
  });
  ForeachEdgeAroundVertex(tail, [&q_bd_edge](HalfedgeEdge_t* e) {
    if (e->pair_->face_ == nullptr) {
      q_bd_edge = e;
    }
  });

  if (p_bd_edge != nullptr && q_bd_edge != nullptr && p_bd_edge != edge
      && q_bd_edge != edge->pair_) {
    return false;
  }

  // Foreach $k$ incident to both i and j, $(i,j,k)$ should be the vertices of a triangle.
  absl::InlinedVector<HalfedgeVertex_t*, 16> i_incidents;
  ForeachEdgeAroundVertex(
      head, [&i_incidents](HalfedgeEdge_t* e) { i_incidents.push_back(e->pair_->vertex_); });
  std::sort(i_incidents.begin(), i_incidents.end());
  absl::InlinedVector<HalfedgeVertex_t*, 16> incidents_both;

  ForeachEdgeAroundVertex(tail, [&i_incidents, &incidents_both](HalfedgeEdge_t* e) {
    if (std::binary_search(i_incidents.begin(), i_incidents.end(), e->pair_->vertex_)) {
      incidents_both.push_back(e->pair_->vertex_);
    }
  });
  return incidents_both.size() <= 2;
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
  AX_DCHECK(std::find_if(vertices_.begin(), vertices_.end(), [vert](const auto& e) {
      return e.get() == vert;
    }) != vertices_.end()) << vert;
  auto beg = vert->halfedge_entry_;
  idx rec_guard = 100;
  do {
    AX_DCHECK(beg != nullptr);
    fn(beg);
    beg = beg->next_->pair_;
    AX_DCHECK(rec_guard-- > 0) << "This vertex has too many edges!";
  } while (beg != vert->halfedge_entry_);
}

void HalfedgeMesh::ForeachEdgeInFace(HalfedgeFace_t* face,
                                     std::function<void(HalfedgeEdge_t*)> const& fn) {
  auto beg = face->halfedge_entry_;
  idx rec_guard = 100;
  do {
    AX_DCHECK(beg != nullptr);
    fn(beg);
    beg = beg->next_;
    AX_DCHECK(rec_guard-- > 0) << "This face has too many edges!";
  } while (beg != face->halfedge_entry_);
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


bool HalfedgeMesh::CheckOk() noexcept {
  absl::flat_hash_set<HalfedgeVertex_t*> vertices;
  for (auto const& v : vertices_) {
    AX_CHECK(v->halfedge_entry_ != nullptr);
    vertices.insert(v.get());
    ForeachEdgeAroundVertex(v.get(), [&](HalfedgeEdge_t* e) {
      AX_CHECK(e->vertex_ == v.get()) << "Edge: " << e << " Vertex: " << e->vertex_;
    });
  }
  absl::flat_hash_set<HalfedgeEdge_t*> edges;
  for (auto const& e : edges_) {
    edges.insert(e.get());
  }
  absl::flat_hash_set<HalfedgeFace_t*> faces{nullptr};
  for (auto const& f : faces_) {
    faces.insert(f.get());
  }
  for (auto const& e : edges_) {
    AX_CHECK(faces.contains(e->face_)) << "Edge: " << e << " Face: " << e->face_;
    AX_CHECK(vertices.contains(e->vertex_));
    AX_CHECK(edges.contains(e->next_));
    AX_CHECK(edges.contains(e->prev_));
    AX_CHECK(edges.contains(e->pair_)) << "Edge: " << e << " Pair: " << e->pair_;
    AX_CHECK(e->pair_->pair_ == e.get());
    AX_CHECK(e->pair_->vertex_ == e->prev_->vertex_);
  }
  
  for (auto const& v : vertices_) {
    AX_CHECK(edges.contains(v->halfedge_entry_)) <<"Vertex: " << v << "Edge: " << v->halfedge_entry_;
  }
  for (auto const& f : faces_) {
    AX_CHECK(edges.contains(f->halfedge_entry_)) << f->halfedge_entry_;
  }
  return true;
}

}  // namespace ax::geo