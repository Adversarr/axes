#pragma once
#include <vector>

#include "axes/core/config.hpp"
#include "axes/math/common.hpp"
#include "axes/math/fields.hpp"
#include "axes/utils/common.hpp"
#include "axes/utils/iota.hpp"
#include "common.hpp"
namespace ax::geo {

struct HalfedgeFace_t;
struct HalfedgeEdge_t;
struct HalfedgeVertex_t;

constexpr idx kHalfedgeInvalidIdx = -1;

struct HalfedgeFace_t {
  HalfedgeEdge_t* halfedge_entry_;         ///< Any halfedge on the face
  idx original_id_ = kHalfedgeInvalidIdx;  ///< Original id from the input mesh
};

struct HalfedgeVertex_t {
  math::vec3r position_;                      ///< Position of the vertex
  idx original_id_ = kHalfedgeInvalidIdx;     ///< Original id from the input mesh
  HalfedgeEdge_t* halfedge_entry_ = nullptr;  ///< Any halfedge on the vertex

  HalfedgeVertex_t() = default;

  explicit HalfedgeVertex_t(math::vec3r const& position, idx original_id = kHalfedgeInvalidIdx)
      : position_(position), original_id_(original_id) {}
};

struct HalfedgeEdge_t {
  HalfedgeEdge_t* next_ = nullptr;
  HalfedgeEdge_t* prev_ = nullptr;
  HalfedgeVertex_t* vertex_ = nullptr;
  HalfedgeFace_t* face_ = nullptr;
  HalfedgeEdge_t* pair_ = nullptr;

  bool IsBoundary() const { return pair_ == nullptr; }

  AX_FORCE_INLINE math::vec3r Normal() const;

  AX_FORCE_INLINE HalfedgeVertex_t* Tail() const { return prev_->vertex_; }

  AX_FORCE_INLINE HalfedgeVertex_t* Head() const { return vertex_; }

  AX_FORCE_INLINE std::pair<HalfedgeVertex_t*, HalfedgeVertex_t*> HeadAndTail() const {
    return std::make_pair(Head(), Tail());
  }
};

namespace details {
struct HalfedgeEdgeOnFaceIterator {
  explicit HalfedgeEdgeOnFaceIterator(HalfedgeFace_t* face)
      : face_(face), current_(face->halfedge_entry_) {}

  using iterator_category = std::forward_iterator_tag;
  using value_type = HalfedgeEdge_t;
  using difference_type = idx;
  using pointer = HalfedgeEdge_t*;
  using reference = HalfedgeEdge_t&;
  using const_reference = HalfedgeEdge_t const&;
  using const_pointer = HalfedgeEdge_t const*;

  AX_FORCE_INLINE HalfedgeEdgeOnFaceIterator& operator++() {
    current_ = current_->next_;
    if (current_ == face_->halfedge_entry_) {
      current_ = nullptr;
    }
    return *this;
  }

  AX_FORCE_INLINE value_type& operator*() const { return *current_; }

  AX_FORCE_INLINE pointer operator->() const { return current_; }

  HalfedgeFace_t* face_;
  HalfedgeEdge_t* current_;
};

struct HalfedgeEdgeOnVertexIterator {
  explicit HalfedgeEdgeOnVertexIterator(HalfedgeVertex_t* vertex)
      : vertex_(vertex), current_(vertex->halfedge_entry_) {}

  using iterator_category = std::forward_iterator_tag;
  using value_type = HalfedgeEdge_t;
  using difference_type = idx;
  using pointer = HalfedgeEdge_t*;
  using reference = HalfedgeEdge_t&;
  using const_reference = HalfedgeEdge_t const&;
  using const_pointer = HalfedgeEdge_t const*;

  AX_FORCE_INLINE HalfedgeEdgeOnVertexIterator& operator++() {
    current_ = current_->pair_->next_;
    if (current_ == vertex_->halfedge_entry_) {
      current_ = nullptr;
    }
    return *this;
  }

  AX_FORCE_INLINE value_type& operator*() const { return *current_; }

  AX_FORCE_INLINE pointer operator->() const { return current_; }

  HalfedgeVertex_t* vertex_;
  HalfedgeEdge_t* current_;
};
}  // namespace details

struct HalfedgeVertexHandle {
  HalfedgeVertex_t* vertex_;
  explicit HalfedgeVertexHandle(HalfedgeVertex_t* vertex) : vertex_(vertex) {}

  AX_FORCE_INLINE details::HalfedgeEdgeOnVertexIterator begin() const {
    return details::HalfedgeEdgeOnVertexIterator(vertex_);
  }

  AX_FORCE_INLINE details::HalfedgeEdgeOnVertexIterator end() const {
    return details::HalfedgeEdgeOnVertexIterator(nullptr);
  }

  AX_FORCE_INLINE explicit operator bool() const { return vertex_ != nullptr; }

  AX_FORCE_INLINE HalfedgeVertex_t* operator->() const { return vertex_; }

  AX_FORCE_INLINE HalfedgeVertex_t& operator*() const { return *vertex_; }
};

struct HalfedgeFaceHandle {
  HalfedgeFace_t* face_;
  explicit HalfedgeFaceHandle(HalfedgeFace_t* face) : face_(face) {}

  AX_FORCE_INLINE details::HalfedgeEdgeOnFaceIterator begin() const {
    return details::HalfedgeEdgeOnFaceIterator(face_);
  }

  AX_FORCE_INLINE details::HalfedgeEdgeOnFaceIterator end() const {
    return details::HalfedgeEdgeOnFaceIterator(nullptr);
  }

  AX_FORCE_INLINE explicit operator bool() const { return face_ != nullptr; }

  AX_FORCE_INLINE HalfedgeFace_t* operator->() const { return face_; }

  AX_FORCE_INLINE HalfedgeFace_t& operator*() const { return *face_; }
};

class HalfedgeMesh {
public:
  HalfedgeMesh() = default;
  AX_DECLARE_COPY_CTOR(HalfedgeMesh, delete);
  AX_DECLARE_MOVE_CTOR(HalfedgeMesh, default);
  /************************* SECT: Constructor and IO *************************/
  HalfedgeMesh(math::field3r const& vertices, math::field3i const& indices);

  AX_NODISCARD SurfaceMesh ToTriangleMesh() const;

  /************************* SECT: Getters *************************/
  AX_NODISCARD HalfedgeFaceHandle GetFace(idx idx) const;
  AX_NODISCARD HalfedgeVertexHandle GetVertex(idx idx) const;

  /************************* SECT: Edge Operation *************************/
  void CollapseEdge(HalfedgeEdge_t* edge, math::vec3r const& target_position);
  AX_NODISCARD bool CheckCollapse(HalfedgeEdge_t* edge);

  HalfedgeEdge_t* TryGetEdgeBetween(HalfedgeVertex_t* v1, HalfedgeVertex_t* v2);

  // Visitors
  void ForeachEdgeAroundVertex(HalfedgeVertex_t* vert,
                               std::function<void(HalfedgeEdge_t*)> const& fn);
  void ForeachEdgeInFace(HalfedgeFace_t* face, std::function<void(HalfedgeEdge_t*)> const& fn);
  void ForeachEdge(std::function<void(HalfedgeEdge_t*)> const& fn);
  void ForeachVertex(std::function<void(HalfedgeVertex_t*)> const& fn);


  /************************* SECT: Metadata Retrival *************************/
  AX_NODISCARD idx NVertices() const noexcept { return static_cast<idx>(vertices_.size()); }
  AX_NODISCARD idx NEdges() const noexcept { return static_cast<idx>(edges_.size()); }
  AX_NODISCARD bool CheckOk() const noexcept;


private:
  void RemoveVertexInternal(HalfedgeVertex_t* vert);
  void RemoveEdgeInternal(HalfedgeEdge_t* edge);
  void RemoveFaceInternal(HalfedgeFace_t* face);
  std::vector<utils::uptr<HalfedgeVertex_t>> vertices_;
  std::vector<utils::uptr<HalfedgeEdge_t>> edges_;
  std::vector<utils::uptr<HalfedgeFace_t>> faces_;
};

math::vec3r HalfedgeEdge_t::Normal() const {
  auto const& A = vertex_->position_;
  auto const& B = next_->vertex_->position_;
  auto const& C = next_->next_->vertex_->position_;
  return (B - A).cross(C - A);
}

}  // namespace ax::geo